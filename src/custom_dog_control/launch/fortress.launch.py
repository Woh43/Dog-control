"""Gazebo Fortress physics and the existing NMPC/WBC effort controller.

No real-hardware plugin, UART node, or hardware IMU driver is launched here.
"""

import atexit
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _after_success(actions, operation):
    def callback(event, _context):
        if event.returncode == 0:
            return actions
        reason = f'{operation} failed: exit {event.returncode}'
        return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]
    return callback


def _setup(context):
    share = Path(get_package_share_directory('custom_dog_control'))
    description = Path(get_package_share_directory('custom_dog_description'))
    runtime = Path(tempfile.mkdtemp(prefix='custom_dog_fortress_'))
    atexit.register(shutil.rmtree, runtime, ignore_errors=True)
    (runtime / 'custom_dog_description').symlink_to(description, target_is_directory=True)
    config_path = LaunchConfiguration('controllers_file').perform(context)
    config = yaml.safe_load(Path(config_path or share / 'config/controllers.yaml').read_text())
    parameters = config['nmpc_wbc_controller']['ros__parameters']
    # Keep AD identity tied to the canonical model, independent of temporary
    # simulator plugin filenames embedded in the rendered robot_description.
    parameters['urdf_file'] = str(description / 'urdf/custom_dog.urdf')
    parameters['robot_description'] = ''
    actuator_model = LaunchConfiguration('actuator_model').perform(context)
    if actuator_model not in ('motor_pd', 'effort'):
        raise ValueError('actuator_model must be motor_pd or effort')
    hardware_mode = 'fortress_hybrid' if actuator_model == 'motor_pd' else 'fortress'
    parameters['hardware_mode'] = 'fortress_hybrid' if actuator_model == 'motor_pd' else 'gazebo'
    parameters['use_sim_time'] = True
    # RK3588 shares CPU time with DART physics. Keep the measured hardware
    # solver budget; controller gains and gait timing remain the baseline.
    parameters['mpc_frequency_hz'] = float(LaunchConfiguration('mpc_frequency_hz').perform(context))
    parameters['mpc_solver_threads'] = int(LaunchConfiguration('mpc_solver_threads').perform(context))
    parameters['use_sim_ground_truth'] = LaunchConfiguration('use_sim_ground_truth').perform(context).lower() == 'true'
    parameters['legacy_joy_layout'] = LaunchConfiguration('legacy_joy_layout').perform(context).lower() == 'true'
    config['controller_manager']['ros__parameters']['use_sim_time'] = True
    task = LaunchConfiguration('task_file').perform(context)
    if task:
        parameters['task_file'] = task
    controllers = runtime / 'controllers.yaml'
    controllers.write_text(yaml.safe_dump(config, sort_keys=False))
    robot = subprocess.run(
        ['xacro', str(share / 'urdf/custom_dog.ros2_control.xacro'),
         f'hardware_mode:={hardware_mode}',
         f'description_urdf:={description / "urdf/custom_dog_gazebo_point_foot.urdf"}',
         f'controllers_file:={controllers}'],
        check=True, capture_output=True, text=True,
    ).stdout
    urdf = runtime / 'robot.urdf'
    urdf.write_text(robot)
    server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-v', '2', LaunchConfiguration('world')],
        output='screen',
    )
    gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'], output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot, value_type=str),
                     'use_sim_time': True}], output='screen',
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                   '/fortress/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        parameters=[{'use_sim_time': True}], output='screen',
    )
    odometry = Node(
        package='custom_dog_control', executable='fortress_odom_adapter.py',
        parameters=[{'use_sim_time': True}], output='screen',
    )
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', LaunchConfiguration('world_name'), '-name', 'custom_dog',
                   '-file', str(urdf), '-x', '0', '-y', '0',
                   '-z', LaunchConfiguration('spawn_z')], output='screen',
    )
    configure = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', 'nmpc_wbc_controller', '--inactive',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '1200',
                   '--service-call-timeout', '1200'], output='screen',
    )
    activate = ExecuteProcess(
        cmd=['ros2', 'control', 'switch_controllers', '--activate',
             'joint_state_broadcaster', 'nmpc_wbc_controller', '--strict',
             '--activate-asap', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    world_name = LaunchConfiguration('world_name').perform(context)
    unpause = ExecuteProcess(
        cmd=['ign', 'service', '-s', f'/world/{world_name}/control',
             '--reqtype', 'ignition.msgs.WorldControl',
             '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000',
             '--req', 'pause: false'], output='screen',
    )
    resource_paths = ':'.join(filter(None, [str(runtime), str(description.parent),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]))
    return [
        SetEnvironmentVariable('IGN_PARTITION', LaunchConfiguration('partition')),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_paths),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ':'.join(filter(None, [
            '/opt/ros/humble/lib', os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')]))),
        LogInfo(msg=f'Fortress runtime files: {runtime}'),
        RegisterEventHandler(OnProcessExit(target_action=spawn,
            on_exit=_after_success([configure], 'Robot spawn'))),
        RegisterEventHandler(OnProcessExit(target_action=configure,
            on_exit=_after_success([activate], 'Controller configuration'))),
        RegisterEventHandler(OnProcessStart(target_action=activate,
            on_start=[TimerAction(period=1.0, actions=[unpause])])),
        RegisterEventHandler(OnProcessExit(target_action=activate,
            on_exit=_after_success([LogInfo(msg='Fortress NMPC/WBC controllers active.')],
                                   'Controller activation'))),
        server, gui, state_publisher, bridge, odometry, spawn,
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('custom_dog_control'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('world', default_value=str(share / 'worlds/flat_fortress.sdf')),
        DeclareLaunchArgument('world_name', default_value='flat_fortress'),
        DeclareLaunchArgument('spawn_z', default_value='0.055'),
        DeclareLaunchArgument('controllers_file', default_value=''),
        DeclareLaunchArgument('actuator_model', default_value='motor_pd'),
        DeclareLaunchArgument('task_file', default_value=''),
        DeclareLaunchArgument('mpc_frequency_hz', default_value='20.0'),
        DeclareLaunchArgument('mpc_solver_threads', default_value='2'),
        DeclareLaunchArgument('use_sim_ground_truth', default_value='false'),
        DeclareLaunchArgument('legacy_joy_layout', default_value='false'),
        DeclareLaunchArgument('partition', default_value='custom_dog_fortress'),
        OpaqueFunction(function=_setup),
    ])
