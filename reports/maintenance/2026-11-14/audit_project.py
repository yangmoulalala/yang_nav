import ast
import json
import re
import subprocess
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path('/home/yang/yang_nav')
EXCLUDED = {'3rdparty', '__pycache__', '.pytest_cache'}
files = sorted(p for base in ('src', 'docs', 'tools')
               for p in (ROOT / base).rglob('*')
               if p.is_file() and not (EXCLUDED & set(p.parts)))
result = {'counts': {}, 'errors': [], 'cpp_without_cmake_mentions': []}

def error(kind, path, detail):
    result['errors'].append({'kind': kind, 'path': str(path.relative_to(ROOT)), 'detail': detail})

markdown = [p for p in files if p.suffix == '.md']
if (ROOT / 'README.md').is_file():
    markdown.append(ROOT / 'README.md')
for relative in ('reports/README.md', 'reports/maintenance/2026-11-14/README.md'):
    if (ROOT / relative).is_file():
        markdown.append(ROOT / relative)
for p in markdown:
    text = p.read_text()
    # Remove fenced and inline examples before looking for Markdown links.
    text = re.sub(r'```.*?```', '', text, flags=re.S)
    text = re.sub(r'`[^`\n]+`', '', text)
    for match in re.finditer(r'!?\[[^\]\n]*\]\(([^\s)]+)(?:\s+"[^"]*")?\)', text):
        target = match[1].strip('<>')
        url = urlsplit(target)
        if url.scheme or target.startswith(('#', '//')):
            continue
        path = (p.parent / unquote(url.path)).resolve()
        if not path.exists():
            error('broken_link', p, target)
        elif url.fragment and path.suffix == '.md':
            slugs = set()
            for heading in re.findall(r'^#{1,6} (.+)$', path.read_text(), flags=re.M):
                heading = re.sub(r'<[^>]+>', '', heading).lower()
                slugs.add(re.sub(r'[^\w -]', '', heading).replace(' ', '-'))
            if unquote(url.fragment) not in slugs:
                error('broken_anchor', p, target)
result['counts']['markdown'] = len(markdown)

python_files = [p for p in files if p.suffix == '.py']
for p in python_files:
    try:
        ast.parse(p.read_text(), filename=str(p))
    except (SyntaxError, UnicodeError) as exc:
        error('python', p, str(exc))
result['counts']['python'] = len(python_files)

shell_files = [p for p in files if p.suffix == '.sh']
for p in shell_files:
    check = subprocess.run(['bash', '-n', str(p)], capture_output=True, text=True)
    if check.returncode:
        error('bash', p, check.stderr)
result['counts']['bash'] = len(shell_files)

import xml.etree.ElementTree as ET
xml_files = [p for p in files if p.suffix in ('.xml', '.urdf', '.sdf')]
for p in xml_files:
    try:
        ET.parse(p)
    except ET.ParseError as exc:
        error('xml', p, str(exc))
result['counts']['xml'] = len(xml_files)

html_files = [p for p in files if p.suffix == '.html']
for p in html_files:
    for index, svg in enumerate(re.findall(r'<svg\b.*?</svg>', p.read_text(), flags=re.S)):
        try:
            ET.fromstring(svg)
        except ET.ParseError as exc:
            error('svg', p, f'svg {index}: {exc}')
result['counts']['html_svg_documents'] = len(html_files)

import yaml
class UniqueLoader(yaml.SafeLoader):
    pass
def unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ValueError(f'duplicate key {key!r}, line {key_node.start_mark.line + 1}')
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping
UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)
yaml_files = [p for p in files if p.suffix in ('.yaml', '.yml')]
for p in yaml_files:
    try:
        yaml.load(p.read_text(), Loader=UniqueLoader)
    except (ValueError, yaml.YAMLError) as exc:
        error('yaml', p, str(exc))
result['counts']['yaml'] = len(yaml_files)

cmake_files = [p for p in files if p.name == 'CMakeLists.txt' or p.suffix == '.cmake']
cmake_text = '\n'.join(p.read_text() for p in cmake_files)
for p in (p for p in files if p.suffix == '.cpp' and 'livox_ros_driver2' not in p.parts):
    if p.name not in cmake_text:
        result['cpp_without_cmake_mentions'].append(str(p.relative_to(ROOT)))
result['counts']['cmake'] = len(cmake_files)

print(json.dumps(result, ensure_ascii=False, indent=2))
raise SystemExit(bool(result['errors']))
