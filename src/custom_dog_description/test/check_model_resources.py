#!/usr/bin/env python3
"""Validate the materialized description without ROS, a simulator, or downloads."""

import argparse
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys
import xml.etree.ElementTree as ET


def require(condition, message):
    if not condition:
        raise ValueError(message)


def values(text, count):
    result = [float(x) for x in text.split()]
    require(len(result) == count and all(math.isfinite(x) for x in result),
            f'Expected {count} finite values: {text}')
    return result


def matrix(inertia):
    x = {k: float(inertia.get(k)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
    require(all(math.isfinite(v) for v in x.values()), 'Nonfinite inertia')
    return [[x['ixx'], x['ixy'], x['ixz']],
            [x['ixy'], x['iyy'], x['iyz']],
            [x['ixz'], x['iyz'], x['izz']]]


def determinant(a):
    return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))


def check_physical_inertia(a, name):
    require(a[0][0] > 0 and a[0][0] * a[1][1] - a[0][1] ** 2 > 0
            and determinant(a) > 0, f'Non-positive-definite inertia: {name}')
    # Half trace(I) * identity - I must be positive semidefinite: the three
    # principal moments must satisfy the physical triangle inequalities.
    half_trace = sum(a[i][i] for i in range(3)) / 2
    covariance = [[(half_trace if i == j else 0) - a[i][j]
                   for j in range(3)] for i in range(3)]
    tolerance = 1e-12
    minors = [covariance[i][i] for i in range(3)]
    minors += [covariance[i][i] * covariance[j][j] - covariance[i][j] ** 2
               for i, j in ((0, 1), (0, 2), (1, 2))]
    require(min(minors) >= -tolerance and determinant(covariance) >= -tolerance,
            f'Unphysical inertia triangle inequality: {name}')


def semantic(element):
    return (element.tag, tuple(sorted(element.attrib.items())),
            tuple(semantic(child) for child in element))


def without_collisions(root):
    result = copy.deepcopy(root)
    for link in result.findall('link'):
        for collision in list(link.findall('collision')):
            link.remove(collision)
    return semantic(result)


def yaml_list(text, section):
    match = re.search(r'^' + re.escape(section) + r':\s*\n((?:  - [^\n]+\n?)+)',
                      text, re.MULTILINE)
    require(match is not None, f'Missing YAML list: {section}')
    return re.findall(r'^  - (\S+)\s*$', match.group(1), re.MULTILINE)


def validate(package):
    manifest = json.loads((package / 'config/model_manifest.json').read_text())
    require(manifest['schema_version'] == 1, 'Unsupported manifest schema')
    require(manifest['upstream']['revision'] ==
            '6c776de8deb76b8fd565cd80f16ac6a4cc8b635d', 'Unexpected source revision')
    require(manifest['model']['loaded_mass_kg'] == 15.15, 'Wrong loaded mass contract')
    require(not any(p.is_symlink() for p in package.rglob('*')),
            'Package contains an external or internal symlink')
    expected = set()
    for directory in ('config', 'launch', 'meshes', 'urdf'):
        expected.update(str(p.relative_to(package)) for p in (package / directory).rglob('*')
                        if p.is_file() and p.name != 'model_manifest.json'
                        # Launch inspection can import Python modules without
                        # changing the installed model payload.
                        and '__pycache__' not in p.relative_to(package).parts
                        and p.suffix not in ('.pyc', '.pyo'))
    expected.update(('package.xml', 'LICENSE.upstream'))
    require(set(manifest['files']) == expected, 'Manifest does not cover the complete payload')
    mesh_triangles = 0
    for relative, entry in manifest['files'].items():
        path = Path(relative)
        require(not path.is_absolute() and '..' not in path.parts, f'Unsafe path: {relative}')
        path = package / path
        require(path.is_file(), f'Missing file: {relative}')
        data = path.read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        require(len(data) == entry['size_bytes'] and digest == entry['sha256'],
                f'File size/SHA-256 mismatch: {relative}')
        if path.suffix.upper() == '.STL':
            require(not data.startswith(b'version https://git-lfs.github.com/spec/v1'),
                    f'LFS pointer instead of an actual mesh: {relative}')
            lfs = entry['upstream_lfs']
            require(digest == lfs['oid_sha256'] and len(data) == lfs['size_bytes'],
                    f'Upstream LFS content mismatch: {relative}')
            require(len(data) >= 84, f'Truncated STL header: {relative}')
            triangles = struct.unpack_from('<I', data, 80)[0]
            require(triangles > 0 and len(data) == 84 + 50 * triangles,
                    f'Invalid binary STL length: {relative}')
            require(all(all(math.isfinite(v) for v in record[:12])
                        for record in struct.iter_unpack('<12fH', memoryview(data)[84:])),
                    f'Nonfinite STL geometry: {relative}')
            mesh_triangles += triangles

    contract = manifest['model']
    roots = {}
    mesh_files = {p.name for p in (package / 'meshes').glob('*.STL')}
    require(len(mesh_files) == 17, 'Expected all 17 CAD meshes')
    for name in manifest['variants']:
        root = ET.parse(package / 'urdf' / name).getroot()
        roots[name] = root
        require(root.tag == 'robot' and root.get('name') == 'custom_dog', f'Wrong robot: {name}')
        links = {x.get('name'): x for x in root.findall('link')}
        joints = root.findall('joint')
        require(len(links) == 17 and len(joints) == 16, f'Incomplete link/joint tree: {name}')
        require(len({x.get('name') for x in joints}) == 16, f'Duplicate joints: {name}')
        actuated = {x.get('name') for x in joints if x.get('type') == 'revolute'}
        require(actuated == set(contract['sdk_joint_order']), f'Wrong actuated joint set: {name}')
        require(all(x.get('type') in ('fixed', 'revolute') for x in joints),
                f'Unexpected joint type: {name}')
        parents = {}
        for joint in joints:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            require(parent in links and child in links and child not in parents,
                    f'Invalid joint parent/child: {name}/{joint.get("name")}')
            parents[child] = parent
            if joint.get('type') == 'revolute':
                axis = values(joint.find('axis').get('xyz'), 3)
                require(abs(sum(x * x for x in axis) - 1) < 1e-12, 'Nonunit joint axis')
                limit = joint.find('limit')
                lower, upper, effort, velocity = [float(limit.get(k))
                                                 for k in ('lower', 'upper', 'effort', 'velocity')]
                require(all(math.isfinite(x) for x in (lower, upper, effort, velocity))
                        and lower < upper and effort > 0 and velocity > 0, 'Invalid joint limits')
        require(set(links) - set(parents) == {'base'}, f'Wrong tree root: {name}')
        for link in links:
            seen = set()
            while link in parents:
                require(link not in seen, f'Joint cycle: {name}')
                seen.add(link)
                link = parents[link]
        mass = 0.0
        referenced_meshes = set()
        for link_name, link in links.items():
            inertial = link.find('inertial')
            link_mass = float(inertial.find('mass').get('value'))
            require(math.isfinite(link_mass) and link_mass > 0, f'Invalid mass: {name}/{link_name}')
            mass += link_mass
            values(inertial.find('origin').get('xyz'), 3)
            values(inertial.find('origin').get('rpy'), 3)
            check_physical_inertia(matrix(inertial.find('inertia')), name + '/' + link_name)
            for mesh in link.findall('.//mesh'):
                uri = mesh.get('filename')
                prefix = 'package://custom_dog_description/meshes/'
                require(uri.startswith(prefix), f'External mesh URI: {uri}')
                filename = uri[len(prefix):]
                require(Path(filename).name == filename and filename in mesh_files,
                        f'Missing or unsafe mesh URI: {uri}')
                referenced_meshes.add(filename)
        require(abs(mass - 15.15) < 1e-10, f'Wrong total mass: {name}: {mass}')
        require(referenced_meshes == mesh_files, f'Incomplete mesh references: {name}')

    canonical = roots['custom_dog.urdf']
    canonical_links = {x.get('name'): x for x in canonical.findall('link')}
    for name, root in roots.items():
        require(without_collisions(root) == without_collisions(canonical),
                f'Variant changes more than collision data: {name}')
        changed = []
        for link in root.findall('link'):
            previous = canonical_links[link.get('name')]
            if tuple(map(semantic, link.findall('collision'))) != tuple(map(semantic, previous.findall('collision'))):
                changed.append(link.get('name'))
        require(sorted(changed) == manifest['variants'][name]['changed_collision_links'],
                f'Unexpected collision-variant changes: {name}')
    point = roots['custom_dog_gazebo_point_foot.urdf']
    expected_removed = {f'{leg}_{part}' for leg in ('FR', 'FL', 'RR', 'RL')
                        for part in ('thigh', 'calf')}
    for link in point.findall('link'):
        before = canonical_links[link.get('name')].findall('collision')
        require((not link.findall('collision') and len(before) == 1)
                if link.get('name') in expected_removed
                else tuple(map(semantic, link.findall('collision'))) == tuple(map(semantic, before)),
                f'Unexpected point-foot collision: {link.get("name")}')

    battery = manifest['battery_adjustment']
    old, payload = battery['unloaded_base'], battery['payload']
    m0, mb = old['mass_kg'], payload['mass_kg']
    require(abs(contract['unloaded_mass_kg'] + mb - 15.15) < 1e-10,
            'Battery and unloaded mass do not compose to the loaded mass')
    m1 = m0 + mb
    expected_com = [(m0 * old['com_m'][i] + mb * payload['center_in_base_m'][i]) / m1
                    for i in range(3)]
    base = canonical_links['base'].find('inertial')
    require(abs(float(base.find('mass').get('value')) - m1) < 1e-10, 'Wrong merged base mass')
    require(max(abs(a - b) for a, b in zip(values(base.find('origin').get('xyz'), 3), expected_com)) < 1e-14,
            'Wrong merged base center of mass')
    require(payload['intrinsic_inertia_model'] == 'point_mass_approximation',
            'Unexpected battery intrinsic inertia assumption')
    expected_inertia = copy.deepcopy(old['inertia_kg_m2'])
    for mass, center in ((m0, old['com_m']), (mb, payload['center_in_base_m'])):
        offset = [center[i] - expected_com[i] for i in range(3)]
        norm_squared = sum(x * x for x in offset)
        for i in range(3):
            for j in range(3):
                expected_inertia[i][j] += mass * ((norm_squared if i == j else 0) - offset[i] * offset[j])
    actual_inertia = matrix(base.find('inertia'))
    require(max(abs(actual_inertia[i][j] - expected_inertia[i][j])
                for i in range(3) for j in range(3)) < 1e-14,
            'Merged base inertia does not satisfy the parallel-axis theorem')
    joint_yaml = (package / 'config/joint_contract.yaml').read_text()
    require(yaml_list(joint_yaml, 'sdk_joint_order') == contract['sdk_joint_order'], 'Joint-order YAML mismatch')
    require(yaml_list(joint_yaml, 'foot_links') == contract['foot_links'], 'Foot-link YAML mismatch')
    battery_yaml = (package / 'config/battery_payload.yaml').read_text()
    for key, expected in (('mass_kg', mb), ('unloaded_robot_mass_kg', contract['unloaded_mass_kg']),
                          ('loaded_robot_mass_kg', 15.15)):
        found = re.search(r'^\s*' + key + r':\s*([^\n]+)', battery_yaml, re.MULTILINE)
        require(found is not None and abs(float(found.group(1)) - expected) < 1e-10,
                f'Battery YAML mismatch: {key}')
    center = re.search(r'^\s*center_in_base_m:\s*(\[[^\n]+\])', battery_yaml, re.MULTILINE)
    require(center is not None and ast.literal_eval(center.group(1)) == payload['center_in_base_m'],
            'Battery-center YAML mismatch')
    package_xml = ET.parse(package / 'package.xml').getroot()
    require(package_xml.findtext('name') == 'custom_dog_description'
            and package_xml.findtext('license') == 'Proprietary', 'Original package identity/license not preserved')
    license_text = (package / 'LICENSE.upstream').read_text()
    require('MIT License' in license_text and 'Copyright (c) 2026 h43' in license_text,
            'Original upstream copyright/license not preserved')
    print(f'MODEL_RESOURCES_PASS: {len(manifest["files"])} locked files, 17 materialized meshes, '
          f'{mesh_triangles} triangles, three equivalent 15.15 kg inertial/joint models')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('package', type=Path)
    args = parser.parse_args()
    try:
        validate(args.package.resolve())
    except (ValueError, KeyError, TypeError, OSError, ET.ParseError) as error:
        print(f'MODEL_RESOURCES_FAILED: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
