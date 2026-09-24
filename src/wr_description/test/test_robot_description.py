import math
import pathlib
import subprocess
import xml.etree.ElementTree as ET


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
XACRO = PACKAGE / 'urdf' / 'wr_chassis.urdf.xacro'
PREFIXES = ('front_left', 'front_right', 'rear_right', 'rear_left')


def expanded_root():
    result = subprocess.run(
        ['xacro', str(XACRO)], check=True, capture_output=True, text=True
    )
    return ET.fromstring(result.stdout)


def test_description_is_one_connected_tree_with_unique_names():
    root = expanded_root()
    links = [node.attrib['name'] for node in root.findall('link')]
    joints = [node.attrib['name'] for node in root.findall('joint')]
    assert len(links) == len(set(links))
    assert len(joints) == len(set(joints))

    children = {joint.find('child').attrib['link'] for joint in root.findall('joint')}
    roots = set(links) - children
    assert roots == {'base_footprint'}
    assert len(joints) == len(links) - 1


def test_four_swerve_modules_and_limits_are_present():
    root = expanded_root()
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    for prefix in PREFIXES:
        steering = joints[f'{prefix}_steering_joint']
        drive = joints[f'{prefix}_wheel_joint']
        assert steering.attrib['type'] == 'revolute'
        assert drive.attrib['type'] == 'continuous'
        limit = steering.find('limit')
        assert math.isclose(float(limit.attrib['lower']), -2.88, abs_tol=1e-8)
        assert math.isclose(float(limit.attrib['upper']), 2.88, abs_tol=1e-8)


def test_dynamic_links_have_inertial_and_meshes_exist():
    root = expanded_root()
    links = {link.attrib['name']: link for link in root.findall('link')}
    for name, link in links.items():
        if name.endswith('_steering_link') or name.endswith('_wheel_link') or name == 'base_link':
            assert link.find('inertial') is not None

    for mesh in root.findall('.//mesh'):
        uri = mesh.attrib['filename']
        assert uri.startswith('package://wr_description/')
        relative = uri.removeprefix('package://wr_description/')
        assert (PACKAGE / relative).is_file(), relative


def test_cad_mass_and_primitive_contact_geometry():
    root = expanded_root()
    dynamic = [
        link for link in root.findall('link')
        if link.attrib['name'] == 'base_link'
        or link.attrib['name'].endswith('_steering_link')
        or link.attrib['name'].endswith('_wheel_link')
    ]
    total_mass = sum(float(link.find('inertial/mass').attrib['value']) for link in dynamic)
    assert math.isclose(total_mass, 20.0, abs_tol=1e-8)

    for link in dynamic:
        collision_meshes = link.findall('collision/geometry/mesh')
        assert collision_meshes == []

    for prefix in PREFIXES:
        wheel = next(link for link in dynamic if link.attrib['name'] == f'{prefix}_wheel_link')
        collisions = {item.attrib['name']: item for item in wheel.findall('collision')}
        tire = collisions['tire_collision'].find('geometry/cylinder')
        roller = collisions['rail_roller_collision'].find('geometry/cylinder')
        assert math.isclose(float(tire.attrib['radius']), 0.0935, abs_tol=1e-9)
        assert math.isclose(float(roller.attrib['radius']), 0.075, abs_tol=1e-9)


def test_required_sensor_frames_are_present():
    root = expanded_root()
    links = {link.attrib['name'] for link in root.findall('link')}
    assert {'imu_link', 'mid360_link', 'front_camera_link', 'rear_camera_link'} <= links
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    rpy = [float(value) for value in joints['mid360_joint'].find('origin').attrib['rpy'].split()]
    assert math.isclose(rpy[1], math.radians(45.0), abs_tol=1e-8)


if __name__ == '__main__':
    test_description_is_one_connected_tree_with_unique_names()
    test_four_swerve_modules_and_limits_are_present()
    test_dynamic_links_have_inertial_and_meshes_exist()
    test_cad_mass_and_primitive_contact_geometry()
    test_required_sensor_frames_are_present()
    print('robot description checks passed')
