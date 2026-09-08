from setuptools import find_packages, setup

package_name = 'catalyst_execute'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/gripper_pose.json',
            'config/catalyst_execute_params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aman',
    maintainer_email='aman@todo.todo',
    description='Execution scripts for the Catalyst Manipulator',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'guide_mode = catalyst_execute.guide_mode:main',
            'sdk_admittance = catalyst_execute.sdk_admittance:main',
            'test_torque_placement = catalyst_execute.test_torque_placement:main',
            'sdk_control_service = catalyst_execute.services.sdk_control_service:main',
            'explore_tag_service = catalyst_execute.services.explore_tag_service:main',
            'compute_poses_service = catalyst_execute.services.compute_poses_service:main',
            'pick_action_server = catalyst_execute.actions.pick_action_server:main',
            'place_action_server = catalyst_execute.actions.place_action_server:main',
            'place_on_robot_action_server = catalyst_execute.actions.place_on_robot_action_server:main',
            'pick_from_robot_container_action_server = catalyst_execute.actions.pick_from_robot_container_action_server:main',
            'explore_action_server = catalyst_execute.actions.explore_action_server:main',
            'detect_well_plate_action_server = catalyst_execute.actions.detect_well_plate_action_server:main',
            'test_eef_bounds_viz = catalyst_execute.test_eef_bounds_viz:main',
        ],
    },
)
