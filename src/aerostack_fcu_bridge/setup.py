from setuptools import find_packages, setup

package_name = 'aerostack_fcu_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ratan',
    maintainer_email='ratan@todo.todo',
    description='Bridge between PX4/MAVLink and AeroStack-RL interfaces',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = aerostack_fcu_bridge.bridge_node:main'
        ],
    },
)
