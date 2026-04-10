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
            'test_execute = catalyst_execute.test_execute:main',
            'guide_mode = catalyst_execute.guide_mode:main',
            'test_apriltag_grasping = catalyst_execute.test_apriltag_grasping:main',
            'plot_grasp_log = catalyst_execute.plot_grasp_log:main',
            'test_admittance = catalyst_execute.test_admittance:main',
            'sdk_admittance = catalyst_execute.sdk_admittance:main',
            'test_place_admittance = catalyst_execute.test_place_admittance:main',
            'pick_and_place = catalyst_execute.pick_and_place:main',
            'test_torque_placement = catalyst_execute.test_torque_placement:main',
            'plot_torque_placement = catalyst_execute.plot_torque_placement:main',
            'ft_data_collection = catalyst_execute.ft_data_collection:main',
            'sdk_control_service = catalyst_execute.services.sdk_control_service:main',
            'explore_tag_service = catalyst_execute.services.explore_tag_service:main',
            'test_modularization = catalyst_execute.test_modularization:main',
            'compute_poses_service = catalyst_execute.services.compute_poses_service:main',
            'pick_action_server = catalyst_execute.actions.pick_action_server:main',
            'place_action_server = catalyst_execute.actions.place_action_server:main',
            'place_on_robot_action_server = catalyst_execute.actions.place_on_robot_action_server:main',
            'pick_from_robot_container_action_server = catalyst_execute.actions.pick_from_robot_container_action_server:main',
            'explore_action_server = catalyst_execute.actions.explore_action_server:main',
            'test_container_place = catalyst_execute.test_container_place:main',
            'test_eef_bounds_viz = catalyst_execute.test_eef_bounds_viz:main',
        ],
    },
)
