import json
import re
from pathlib import Path
import yaml

root = Path('/home/yang/yang_nav/src/z-localization')
out = {}
for module, node in [('frontend', 'zlio'), ('backend', 'backend_node')]:
    data = yaml.safe_load((root/module/'config/example.yaml').read_text())[node]['ros__parameters']
    leaves = {}
    def walk(prefix, value):
        if isinstance(value, dict):
            for key, child in value.items():
                walk(f'{prefix}.{key}' if prefix else key, child)
        else:
            leaves[prefix] = value
    walk('', data)
    documented = {}
    group = ''
    for line in (root/module/'docs/parameters.md').read_text().splitlines():
        match = re.match(r'#### `([^`]+)`', line)
        if match:
            group = match[1]
        row = re.match(r'\| `([^`]+)` \| [^|]+ \| `([^`]+)` \|', line)
        if row:
            name = row[1] if row[1].startswith(group + '.') else group + '.' + row[1]
            documented[name] = row[2]
    mismatches = []
    for name in leaves.keys() & documented.keys():
        try:
            value = yaml.safe_load(documented[name])
            if isinstance(leaves[name], (int, float)) and not isinstance(leaves[name], bool):
                try:
                    value = float(documented[name])
                except ValueError:
                    pass
            if value != leaves[name]:
                mismatches.append({'key': name, 'example': leaves[name], 'doc': documented[name]})
        except yaml.YAMLError:
            pass
    out[module] = {'missing': sorted(leaves.keys() - documented.keys()), 'mismatches': mismatches}
print(json.dumps(out, ensure_ascii=False, indent=2))
