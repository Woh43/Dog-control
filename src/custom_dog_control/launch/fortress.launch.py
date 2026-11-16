"""Gazebo Fortress physics and the existing NMPC/WBC effort controller.

No real-hardware plugin, UART node, or hardware IMU driver is launched here.
"""

import atexit
import math
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


def _simulation_contact_confidence(parameters, custom_profile, actuator_model,
                                 command_delay_s, delay_override='', ramp_override=''):
    # Only the default simulation profile derives confidence timing from the
    # configured extra delay. A custom controllers_file retains its own values.
    # This does not identify contact or motor latency on real hardware.
    if delay_override.strip():
        delay = float(delay_override)
    elif custom_profile:
        delay = float(parameters.get('estimator_planned_contact_delay_s', 0.0))
    else:
        delay = command_delay_s if actuator_model == 'motor_pd' else 0.0
    if ramp_override.strip():
        ramp = float(ramp_override)
    elif custom_profile:
        ramp = float(parameters.get('estimator_planned_contact_ramp_s', 0.0))
    else:
        ramp = 0.010 if delay > 0.0 else 0.0
    for name, value in (('delay', delay), ('ramp', ramp)):
        if not math.isfinite(value) or not 0.0 <= value <= 0.1:
            raise ValueError(f'simulation estimator planned contact {name} must be in [0, 0.1] s')
    return {'estimator_planned_contact_delay_s': delay,
            'estimator_planned_contact_ramp_s': ramp}


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
    simulation = {name: LaunchConfiguration(name).perform(context) for name in (
        'simulation_imu_rate_hz', 'simulation_command_delay_s',
        'simulation_coulomb_friction', 'simulation_viscous_friction',
        'simulation_friction_velocity_scale')}
    imu_rate = float(simulation['simulation_imu_rate_hz'])
    delay = float(simulation['simulation_command_delay_s'])
    velocity_scale = float(simulation['simulation_friction_velocity_scale'])
    if not math.isfinite(imu_rate) or not 0 < imu_rate <= 1000:
        raise ValueError('simulation_imu_rate_hz must be in (0, 1000] Hz')
    if not math.isfinite(delay) or not 0 <= delay <= 0.05:
        raise ValueError('simulation_command_delay_s must be in [0, 0.05] s')
    if not math.isfinite(velocity_scale) or velocity_scale <= 0:
        raise ValueError('simulation_friction_velocity_scale must be positive and finite')
    nonideal_motor = delay > 0
    for name in ('simulation_coulomb_friction', 'simulation_viscous_friction'):
        values = [float(value) for value in simulation[name].split()]
        if len(values) not in (1, 3, 12) or any(not math.isfinite(v) or v < 0 for v in values):
            raise ValueError(f'{name} requires 1, 3 or 12 nonnegative finite values')
        nonideal_motor |= any(value > 0 for value in values)
    if actuator_model != 'motor_pd' and nonideal_motor:
        raise ValueError('simulation motor delay/friction require actuator_model:=motor_pd')
    hardware_mode = 'fortress_hybrid' if actuator_model == 'motor_pd' else 'fortress'
    parameters['hardware_mode'] = 'fortress_hybrid' if actuator_model == 'motor_pd' else 'gazebo'
    parameters['use_sim_time'] = True
    parameters.update(_simulation_contact_confidence(
        parameters, bool(config_path), actuator_model, delay,
        LaunchConfiguration('simulation_estimator_planned_contact_delay_s').perform(context),
        LaunchConfiguration('simulation_estimator_planned_contact_ramp_s').perform(context)))
    update_rate = float(config['controller_manager']['ros__parameters']['update_rate'])
    if not math.isfinite(update_rate) or update_rate <= 0.0:
        raise ValueError('controller update_rate must be finite and positive')
    budget_override = LaunchConfiguration('simulation_motor_pd_budget_horizon_s').perform(context)
    if budget_override.strip():
        budget_horizon = float(budget_override)
    elif config_path:
        budget_horizon = float(parameters.get('motor_pd_budget_horizon_s', 0.0))
    else:
        # The signed model remains opt-in until it passes the motion envelope.
        # A proposed 20 ms experiment uses H=25 ms, but A is not identified.
        budget_horizon = float(parameters.get('motor_pd_budget_horizon_s', 0.0))
    if not math.isfinite(budget_horizon) or not 0.0 <= budget_horizon <= 0.1:
        raise ValueError('simulation motor PD budget horizon must be in [0, 0.1] s')
    if actuator_model != 'motor_pd' and budget_horizon > 0.0:
        raise ValueError('a signed motor PD delivery window requires actuator_model:=motor_pd')
    parameters['motor_pd_budget_horizon_s'] = budget_horizon
    nominal_override = LaunchConfiguration('simulation_motor_pd_nominal_execution_time_s').perform(context)
    if nominal_override.strip():
        nominal_time = float(nominal_override)
    elif config_path:
        nominal_time = float(parameters.get('motor_pd_nominal_execution_time_s', 0.0))
    else:
        nominal_time = float(parameters.get('motor_pd_nominal_execution_time_s', 0.0))
    if not math.isfinite(nominal_time) or not 0.0 <= nominal_time <= budget_horizon:
        raise ValueError('nominal motor PD execution time must lie in [0, budget horizon]')
    parameters['motor_pd_nominal_execution_time_s'] = nominal_time
    parameters['imu_orientation_prediction_max_s'] = float(
        LaunchConfiguration('imu_orientation_prediction_max_s').perform(context))
    # Explicit launch overrides also apply to custom controller profiles.
    # Simulation-time pacing does not certify onboard wall-clock deadlines.
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
         f'controllers_file:={controllers}',
         f'motor_pd_signed_budget:={"true" if budget_horizon > 0.0 else "false"}',
         f'motor_pd_budget_horizon_s:={budget_horizon}',
         f'motor_pd_nominal_execution_time_s:={nominal_time}',
         'motor_pd_assumed_acceleration_rad_s2:=' + ' '.join(str(value) for value in
             parameters.get('motor_pd_assumed_acceleration_rad_s2', [100.0])),
         *(f'{name}:={value}' for name, value in simulation.items())],
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
        DeclareLaunchArgument('simulation_imu_rate_hz', default_value='100.0',
            description='Simulated IMU publication rate; current physical IMU is 100 Hz'),
        # Prediction remains opt-in: the initial held-attitude failure did not
        # reproduce consistently, and hardware AHRS/gyro alignment is unverified.
        DeclareLaunchArgument('imu_orientation_prediction_max_s', default_value='0.0',
            description='Bounded gyro prediction of attitude to the control timestamp; 0 disables'),
        # Assumed hardware stress profile, not identified actuator parameters.
        # Hip/thigh added friction is zero; all calves use the same coefficients.
        DeclareLaunchArgument('simulation_command_delay_s', default_value='0.005',
            description='Additional motor command delay beyond one outer control cycle, in seconds'),
        DeclareLaunchArgument('simulation_motor_pd_budget_horizon_s', default_value='',
            description='Opt-in signed FF+PD window in seconds; blank preserves the profile; '
                        '0 retains legacy budgeting; not an identified actuator bound'),
        DeclareLaunchArgument('simulation_motor_pd_nominal_execution_time_s', default_value='',
            description='Opt-in execution PD center D in [0,H]; blank preserves the profile'),
        DeclareLaunchArgument('simulation_estimator_planned_contact_delay_s', default_value='',
            description='Planned-contact confidence delay in [0,0.1] s; blank derives from '
                        'motor delay for the default profile and preserves custom controllers_file'),
        DeclareLaunchArgument('simulation_estimator_planned_contact_ramp_s', default_value='',
            description='Planned-contact confidence ramp in [0,0.1] s; blank uses 0.010 s '
                        'after a positive default-profile delay, or preserves custom controllers_file'),
        DeclareLaunchArgument('simulation_coulomb_friction',
            default_value='0 0 0.45',
            description='Joint-side friction load in Nm: scalar, hip/thigh/calf, or 12 joint values'),
        DeclareLaunchArgument('simulation_viscous_friction',
            default_value='0 0 0.0175',
            description='Joint-side viscous load in Nm s/rad: scalar, 3 or 12 values'),
        DeclareLaunchArgument('simulation_friction_velocity_scale', default_value='0.05',
            description='Positive smoothing velocity for simulated Coulomb friction, in rad/s'),
        DeclareLaunchArgument('task_file', default_value=''),
        DeclareLaunchArgument('mpc_frequency_hz', default_value='50.0'),
        DeclareLaunchArgument('mpc_solver_threads', default_value='2'),
        DeclareLaunchArgument('use_sim_ground_truth', default_value='false'),
        DeclareLaunchArgument('legacy_joy_layout', default_value='false'),
        DeclareLaunchArgument('partition', default_value='custom_dog_fortress'),
        OpaqueFunction(function=_setup),
    ])
