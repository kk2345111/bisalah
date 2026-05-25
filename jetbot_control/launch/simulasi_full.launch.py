"""
================================================================
simulasi_full.launch.py  —  Launch File Simulasi Lengkap  [FIXED]
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
PERUBAHAN DARI VERSI LAMA:
  [FIX-LAUNCH-1] Tambah parameter send_host untuk labview_bridge
    → Default: 'host.docker.internal' (resolve ke Windows host)
    → Override: ros2 launch ... send_host:=172.17.0.1

  [FIX-LAUNCH-2] Tambah argumen send_host sebagai launch argument

TOPOLOGI TOPIC (DIPERBARUI):
  trajectory_node ─── /reference_state [6 val] ──► kbbc_node
  kbbc_node ──────── /system_ready ──────────────► trajectory_node
  plant_node ──────── /robot_state ──────────────► kbbc_node
  plant_node ──────── /wheel_speed_plant ─────────► pid_node
  kbbc_node ──────── /wheel_cmd ─────────────────► pid_node
  pid_node ───────── /motor_voltage ─────────────► plant_node
  labview_bridge ─── /labview_params ────────────► pid_node
  labview_bridge ─── /sim_control ───────────────► semua node

CARA JALANKAN:
  ros2 launch jetbot_control simulasi_full.launch.py

  # Dengan KBBC+PID:
  ros2 launch jetbot_control simulasi_full.launch.py sim_type:=1

  # Dengan IP host custom (jika host.docker.internal tidak jalan):
  ros2 launch jetbot_control simulasi_full.launch.py send_host:=172.17.0.1
================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    sim_type_arg = DeclareLaunchArgument(
        'sim_type', default_value='0',
        description='0=KBBC Only, 1=KBBC+PID')
    kp_arg = DeclareLaunchArgument(
        'kp', default_value='0.1', description='PID proportional gain')
    ki_arg = DeclareLaunchArgument(
        'ki', default_value='0.02', description='PID integral gain')
    kd_arg = DeclareLaunchArgument(
        'kd', default_value='0.005', description='PID derivative gain')
    # [FIX-LAUNCH-2] Tambah send_host argument
    send_host_arg = DeclareLaunchArgument(
        'send_host', default_value='host.docker.internal',
        description='IP host LabVIEW (Windows). '
                    'Gunakan host.docker.internal atau 172.17.0.1')

    sim_type  = LaunchConfiguration('sim_type')
    kp        = LaunchConfiguration('kp')
    ki        = LaunchConfiguration('ki')
    kd        = LaunchConfiguration('kd')
    send_host = LaunchConfiguration('send_host')

    # Node 1: Trajectory Generator (tunggu /system_ready)
    trajectory_node = Node(
        package='jetbot_control',
        executable='trajectory_node',
        name='trajectory_node',
        parameters=[{
            'scale_k': 1.5,
            'omega':   0.1,
            'dt':      0.01,
        }],
        output='screen',
    )

    # Node 2: Robot Plant (selalu publish robot_state)
    plant_node = Node(
        package='jetbot_control',
        executable='plant_node',
        name='plant_node',
        parameters=[{
            'x0':  0.0,
            'y0':  0.0,
            'th0': 0.0,
            'dt':  0.01,
        }],
        output='screen',
    )

    # Node 3: KBBC Controller (kirim /system_ready ke trajectory)
    kbbc_node = Node(
        package='jetbot_control',
        executable='kbbc_node',
        name='kbbc_node',
        parameters=[{
            'Kx':      0.5,
            'Ky':      3.0,
            'Ktheta':  0.5,
            'Kdirect': 0.3,
            'dt':      0.01,
        }],
        output='screen',
    )

    # Node 4: PID Controller (tidak publish volt=0 saat idle)
    pid_node = Node(
        package='jetbot_control',
        executable='pid_node',
        name='pid_node',
        parameters=[{
            'kp_R':     kp,
            'ki_R':     ki,
            'kd_R':     kd,
            'kp_L':     kp,
            'ki_L':     ki,
            'kd_L':     kd,
            'sim_type': sim_type,
            'dt':       0.01,
        }],
        output='screen',
    )

    # Node 5: Sensor Model
    sensor_node = Node(
        package='jetbot_control',
        executable='sensor_node',
        name='sensor_node',
        parameters=[{'dt': 0.01}],
        output='screen',
    )

    # Node 6: Logger CSV
    logger_node = Node(
        package='jetbot_control',
        executable='logger_node',
        name='logger_node',
        parameters=[{
            'debug_interval': 1.0,
            'dt':             0.01,
        }],
        output='screen',
    )

    # Node 7: LabVIEW Bridge (fixed IP + 100Hz send + rate-limited log)
    labview_bridge = Node(
        package='jetbot_control',
        executable='labview_bridge',
        name='labview_bridge',
        parameters=[{
            'listen_port': 5052,
            'send_port':   5053,
            'send_host':   send_host,   # [FIX-LAUNCH-1]
        }],
        output='screen',
    )

    return LaunchDescription([
        sim_type_arg, kp_arg, ki_arg, kd_arg,
        send_host_arg,            # [FIX-LAUNCH-2]
        trajectory_node,
        plant_node,
        kbbc_node,
        pid_node,
        sensor_node,
        logger_node,
        labview_bridge,
    ])
