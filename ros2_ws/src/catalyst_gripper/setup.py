from setuptools import find_packages, setup

package_name = 'catalyst_gripper'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/gripper_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aman',
    maintainer_email='aman@todo.todo',
    description='Dynamixel AX-18A gripper driver for Catalyst Manipulator',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_node = catalyst_gripper.gripper_node:main',
        ],
    },
)
