from setuptools import find_packages, setup

package_name = 'moveit_path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='sanjay',
    maintainer_email='sanjay.ramnauth@mail.utoronto.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Replace with your Python node entry point
            'example_action_client = moveit_path_planning.plan_execute_client_example:main',
        ],
    },
)
