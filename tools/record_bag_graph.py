import os
from pathlib import Path
import sys
import time

from rcl_interfaces.srv import GetParameters, ListParameters
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import parameter_value_to_python
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
import yaml


DISCOVERY_SECONDS = 1.25
SNAPSHOT_SECONDS = 5.0


def spin_until(executor, deadline):
    remaining = deadline - time.monotonic()
    if remaining > 0:
        executor.spin_once(timeout_sec=min(0.05, remaining))


def graph_list(entries):
    return ''.join(f'{name} [{", ".join(types)}]\n' for name, types in sorted(entries)
                   if not topic_or_service_is_hidden(name))


def write_parameters(directory, index, name, names, response):
    if len(names) != len(response.values):
        raise ValueError('parameter response length does not match the request')
    parameters = {key: parameter_value_to_python(value) for key, value in zip(names, response.values)}
    path = directory / f'params_{index}_{name.strip("/").replace("/", "_")}.yaml'
    path.write_text(yaml.safe_dump({name: {'ros__parameters': parameters}}, allow_unicode=True))


def snapshot(directory, node, executor):
    started = time.monotonic()
    deadline = started + SNAPSHOT_SECONDS
    while time.monotonic() < started + DISCOVERY_SECONDS:
        spin_until(executor, started + DISCOVERY_SECONDS)

    names = {f'{namespace.rstrip("/")}/{name}' for name, namespace in node.get_node_names_and_namespaces()}
    nodes = sorted(names - {node.get_fully_qualified_name()})
    nodes = [name for name in nodes if not topic_or_service_is_hidden(name)]
    services = node.get_service_names_and_types()
    (directory / 'nodes.txt').write_text(''.join(f'{name}\n' for name in nodes))
    (directory / 'topics.txt').write_text(graph_list(node.get_topic_names_and_types()))
    (directory / 'services.txt').write_text(graph_list(services))

    advertised = {name for name, _ in services}
    waiting = {}
    listing = {}
    reading = {}
    skipped = {}
    saved = 0
    for index, name in enumerate(nodes):
        list_service = f'{name}/list_parameters'
        get_service = f'{name}/get_parameters'
        if list_service in advertised and get_service in advertised:
            waiting[index] = (node.create_client(ListParameters, list_service),
                              node.create_client(GetParameters, get_service))
        else:
            skipped[name] = 'parameter services not discovered'

    while (waiting or listing or reading) and time.monotonic() < deadline:
        for index, (list_client, get_client) in list(waiting.items()):
            if list_client.service_is_ready() and get_client.service_is_ready():
                del waiting[index]
                try:
                    listing[index] = (list_client.call_async(ListParameters.Request()), get_client)
                except Exception as error:
                    skipped[nodes[index]] = str(error)
        for index, (future, get_client) in list(listing.items()):
            if future.done():
                del listing[index]
                try:
                    names = sorted(future.result().result.names)
                    reading[index] = (get_client.call_async(GetParameters.Request(names=names)), names)
                except Exception as error:
                    skipped[nodes[index]] = str(error)
        for index, (future, names) in list(reading.items()):
            if future.done():
                del reading[index]
                try:
                    write_parameters(directory, index, nodes[index], names, future.result())
                    saved += 1
                except (RuntimeError, ValueError) as error:
                    skipped[nodes[index]] = str(error)
        if waiting or listing or reading:
            spin_until(executor, deadline)

    for index in waiting.keys() | listing.keys() | reading.keys():
        skipped[nodes[index]] = 'snapshot deadline reached'
    (directory / 'snapshot.yaml').write_text(yaml.safe_dump({'saved': saved, 'skipped': skipped}))
    print(f'{directory.name}: saved parameters for {saved}/{len(nodes)} nodes '
          f'in {time.monotonic() - started:.1f} s', flush=True)


def main(directory):
    directory.mkdir(exist_ok=True)
    rclpy.init(args=[])
    try:
        node = rclpy.create_node(f'_bag_snapshot_{os.getpid()}', enable_rosout=False,
                                 start_parameter_services=False)
        try:
            with SingleThreadedExecutor() as executor:
                executor.add_node(node)
                snapshot(directory, node, executor)
        finally:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(Path(sys.argv[1]))
