from setuptools import find_packages, setup

package_name = 'auto_keyboard_demo'

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
    maintainer='nazib',
    maintainer_email='49647912+nazib007@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_pub = auto_keyboard_demo.publisher_node:main',
            'keyboard_sub = auto_keyboard_demo.subscriber_node:main',
        ],
    },
)
