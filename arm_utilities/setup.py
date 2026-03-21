from setuptools import find_packages, setup
import os
import sysconfig

package_name = 'arm_utilities'

# Resolves to e.g. /home/.../install/arm_utilities/lib/python3.10/site-packages
site_packages = sysconfig.get_path('purelib')
# We only want the relative portion: lib/python3.10/site-packages
rel_site_packages = os.path.relpath(site_packages, sysconfig.get_path('data'))

def get_yaml_data_files(subpackage: str) -> list:
    data_files = []
    base_dir = os.path.join(package_name, subpackage)
    for root, _, files in os.walk(base_dir):
        matched = [os.path.join(root, f) for f in files if f.endswith(('.yaml', '.yml'))]
        if matched:
            install_dir = os.path.join(rel_site_packages, root)
            data_files.append((install_dir, matched))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *get_yaml_data_files('arm_configs'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunny-rsx',
    maintainer_email='sunny-rsx@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)