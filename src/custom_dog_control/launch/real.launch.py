import atexit
import os
from pathlib import Path
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable,
    SetLaunchConfiguration, TimerAction,
)
from launch.conditions import IfCondition
from launch.logging import launch_config
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _configure_controllers(context, common_file, real_file, canonical_urdf):
    config = yaml.safe_load(Path(common_file).read_text())
    overrides = yaml.safe_load(Path(real_file).read_text())
    for node_name, node in overrides.items():
        config.setdefault(node_name, {}).setdefault('ros__parameters', {}).update(
            node['ros__parameters'])
    parameters = config['nmpc_wbc_controller']['ros__parameters']
    # Hardware parameters and temporary robot_description paths do not change
    # the model dynamics. Use the same canonical model as the physics tests.
    parameters['urdf_file'] = canonical_urdf
    parameters['hardware_mode'] = 'real'
    parameters['use_sim_ground_truth'] = False
    task_file = LaunchConfiguration('task_file').perform(context)
    if task_file:
        task = Path(task_file).expanduser().resolve(strict=True)
        if not task.is_file():
            raise ValueError(f'NMPC task is not a regular file: {task}')
        parameters['task_file'] = str(task)
    runtime = Path(tempfile.mkdtemp(prefix='custom_dog_real_'))
    atexit.register(shutil.rmtree, runtime, ignore_errors=True)
    path = runtime / 'controllers.yaml'
    path.write_text(yaml.safe_dump(config, sort_keys=False))
    return [SetLaunchConfiguration('real_controllers_file', str(path))]


def generate_launch_description():
    package_share = get_package_share_directory('custom_dog_control')
    description_share = get_package_share_directory('custom_dog_description')
    controllers = os.path.join(package_share, 'config', 'controllers.yaml')
    real_override = os.path.join(
        package_share, 'config', 'real_controller.yaml')
    xacro_file = os.path.join(
        package_share, 'urdf', 'custom_dog.ros2_control.xacro')
    canonical_urdf = os.path.join(
        description_share, 'urdf', 'custom_dog.urdf')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' hardware_mode:=real',
            ' description_urdf:=', canonical_urdf,
            ' startup_alignment_enabled:=',
            LaunchConfiguration('startup_alignment_enabled'),
            ' physical_estop_verified:=',
            LaunchConfiguration('physical_estop_verified'),
            ' require_physical_estop_confirmation:=',
            LaunchConfiguration('require_physical_estop_confirmation'),
            ' fr_port:=', LaunchConfiguration('fr_port'),
            ' fl_port:=', LaunchConfiguration('fl_port'),
            ' rr_port:=', LaunchConfiguration('rr_port'),
            ' rl_port:=', LaunchConfiguration('rl_port'),
            ' motor_directions:="', LaunchConfiguration('motor_directions'), '"',
            ' parallel_leg_io:=', LaunchConfiguration('parallel_leg_io'),
            ' calibration_hip_deg:=',
            LaunchConfiguration('calibration_hip_deg'),
            ' calibration_thigh_deg:=',
            LaunchConfiguration('calibration_thigh_deg'),
            ' calibration_calf_deg:=',
            LaunchConfiguration('calibration_calf_deg'),
        ]),
        value_type=str,
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            LaunchConfiguration('real_controllers_file'),
            {'robot_description': robot_description},
        ],
        output='both',
    )
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='both',
    )
    imu = Node(
        package='fdilink_ahrs',
        executable='ahrs_driver_node',
        name='ahrs_driver',
        condition=IfCondition(LaunchConfiguration('start_imu')),
        parameters=[{
            'if_debug_': False,
            'serial_port_': LaunchConfiguration('imu_port'),
            'serial_baud_': ParameterValue(
                LaunchConfiguration('imu_baud'), value_type=int),
            'imu_topic': '/imu',
            'imu_frame_id_': 'base',
            'device_type_': 1,
        }],
        output='both',
    )
    joy = Node(
        package='custom_dog_control',
        executable='event2joy.py',
        name='event2joy_node',
        condition=IfCondition(LaunchConfiguration('start_joy')),
        parameters=[{
            'event_path': LaunchConfiguration('joy_event_path'),
            'publish_hz': 250.0,
        }],
        output='both',
    )
    load_controllers = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager-timeout', '1200',
                    '--service-call-timeout', '1200',
                ],
                output='both',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                # A new URDF/task fingerprint compiles CppAD libraries during
                # configure. On ARM64 this can take several minutes; the
                # spawner must not abandon activation after its default 10 s.
                arguments=[
                    'nmpc_wbc_controller',
                    '--controller-manager-timeout', '1200',
                    '--service-call-timeout', '1200',
                ],
                output='both',
            ),
        ],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            package_share, 'rviz', 'custom_dog_real.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='both',
    )
    diagnostics_recorder = Node(
        package='custom_dog_control',
        executable='record_control_diagnostics.py',
        condition=IfCondition(LaunchConfiguration('record_diagnostics')),
        parameters=[{'output_file': os.path.join(
            launch_config.log_dir, 'control-diagnostics.jsonl')}],
        output='both',
    )

    return LaunchDescription([
        # Keep SDK stdout and ROS/Python diagnostics in launch.log as well as
        # the desktop terminal; a screen-only launch loses the failure context.
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
        SetEnvironmentVariable('PYTHONUNBUFFERED', '1'),
        DeclareLaunchArgument(
            'physical_estop_verified', default_value='false'),
        DeclareLaunchArgument(
            'require_physical_estop_confirmation', default_value='true',
            description='Require manual confirmation before activation; not an E-stop input'),
        DeclareLaunchArgument('task_file', default_value=''),
        DeclareLaunchArgument('startup_alignment_enabled', default_value='false'),
        DeclareLaunchArgument('fr_port', default_value='/dev/ttyS3'),
        DeclareLaunchArgument('fl_port', default_value='/dev/ttyS4'),
        DeclareLaunchArgument('rr_port', default_value='/dev/ttyS7'),
        DeclareLaunchArgument('rl_port', default_value='/dev/ttyS8'),
        DeclareLaunchArgument(
            'motor_directions',
            default_value='1 -1 1 1 1 -1 -1 -1 1 -1 1 -1',
            description='URDF signs, FR/FL/RR/RL hip/thigh/calf; verify by hand'),
        DeclareLaunchArgument('parallel_leg_io', default_value='true'),
        DeclareLaunchArgument('calibration_hip_deg', default_value='0.0'),
        DeclareLaunchArgument('calibration_thigh_deg', default_value='71.8'),
        DeclareLaunchArgument('calibration_calf_deg', default_value='-161.8'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_baud', default_value='921600'),
        DeclareLaunchArgument('joy_event_path', default_value='auto'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('start_joy', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'record_diagnostics', default_value='true',
            description='Save controller diagnostics beside launch.log'),
        OpaqueFunction(
            function=_configure_controllers,
            args=[controllers, real_override, canonical_urdf]),
        control_node,
        state_publisher,
        imu,
        joy,
        diagnostics_recorder,
        load_controllers,
        rviz,
    ])
