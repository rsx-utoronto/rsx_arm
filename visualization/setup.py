from setuptools import find_packages, setup

package_name = 'visualization'

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
    description='ROS2 arm visualization package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_viz = visualization.arm_viz:main',
            'cam_compress = visualization.cam_compress:main',
            'cam_conv = visualization.cam_conv:main',
        ],
    },
)