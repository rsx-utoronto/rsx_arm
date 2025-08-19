from setuptools import find_packages, setup

package_name = 'gui'

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
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_gui = gui.qt5_gui:main'
        ],
    },
    scripts=[
        'scripts/run_gui.bash',
    ],
)
