from setuptools import find_packages, setup
import glob
import os

package_name = 'simulation'
stl_directories = {}
for x in os.walk('meshes'):
    stl_directories[x[0]] = glob.glob(os.path.join(x[0], '*.STL'))

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/urdfs',
     ['urdfs/arm_circ_2024.urdf', 'urdfs/arm_urdf.rviz', 'urdfs/Arm_URDF_2025.urdf']),
]
for dir in stl_directories.keys():
    for file in stl_directories[dir]:
        data_files.append(('share/' + package_name + '/' + dir, [file]))
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanjay',
    maintainer_email='sanjay.ramnauth@mail.utoronto.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
