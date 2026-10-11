from setuptools import find_packages, setup

package_name = 'kb_joy_emu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanjay',
    maintainer_email='sanjay.ramnauth@mail.utoronto.ca',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kb_joy_emulator = kb_joy_emu.kb_joy_emulator:main'
        ],
    },
)
