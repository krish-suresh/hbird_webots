from setuptools import setup
import os, glob

package_name = 'hbird_webots'
submodules = 'hbird_webots/scripts'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/mp2.launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/demo.launch.py']))
data_files.append(('share/' + package_name + '/protos', ['protos/Mark2Assembly.proto']))
data_files.append(('share/' + package_name + '/protos/meshes', ['protos/meshes/base_link_good.obj']))
data_files.append(('share/' + package_name + '/protos/meshes', ['protos/meshes/base_link_good.mtl']))
data_files.append(('share/' + package_name + '/protos/meshes', ['protos/meshes/rackwithtexture.obj']))
data_files.append(('share/' + package_name + '/protos/meshes', ['protos/meshes/rackwithtexture.mtl']))
data_files.append(('share/' + package_name + '/protos/meshes', ['protos/meshes/propeller.obj']))
data_files.append(('share/' + package_name + '/protos/meshes', ['protos/meshes/propeller.mtl']))
data_files.append(('share/' + package_name + '/protos/textures', ['protos/textures/cotton beige.jpg']))
data_files.append(('share/' + package_name + '/protos/textures', ['protos/textures/fast_helix.png']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/flight_arena.wbt']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/LPB.wbt']))
data_files.append(('share/' + package_name + '/resource', [
    'resource/hbird_drone.urdf'
]))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Olin HAIR Lab',
    maintainer_email='gaby.blake2@gmail.com',
    license='TODO: License declaration',
    tests_require=['pytest'],
    description='Hbird drone robot ROS2 interface for Webots.',
    entry_points={
        'console_scripts': [
            # 'agent_control_node = hbird_webots.agent_control_node:main',
            'hbird_sim_node = hbird_webots.hbird_sim_node:main'
        ],
        # 'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
