"""
================================================================
master.launch.py  —  Launch Master Node + LabVIEW Bridge
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
Arsitektur baru: hanya 2 node (bukan 7):
  1. master_node   — seluruh simulasi dalam 1 node
  2. labview_bridge — hanya untuk UDP bridge ke LabVIEW

Cara jalankan:
  ros2 launch jetbot_control master.launch.py

  # Dengan KBBC+PID default:
  ros2 launch jetbot_control master.launch.py sim_type:=1

  # Custom IP LabVIEW:
  ros2 launch jetbot_control master.launch.py send_host:=172.17.0.1

Monitor simulasi:
  ros2 topic echo /master/robot_state
  ros2 topic echo /master/error_state
  ros2 topic echo /master/debug_info

RESET/STOP manual:
  ros2 topic pub /sim_control std_msgs/msg/Int32 '{data: 9}'   # RESET
  ros2 topic pub /sim_control std_msgs/msg/Int32 '{data: 99}'  # STOP
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
        'kp', default_value='0.1', description='Kp PID awal')
    ki_arg = DeclareLaunchArgument(
        'ki', default_value='0.02', description='Ki PID awal')
    kd_arg = DeclareLaunchArgument(
        'kd', default_value='0.005', description='Kd PID awal')
    send_host_arg = DeclareLaunchArgument(
        'send_host', default_value='host.docker.internal',
        description='IP host LabVIEW. Gunakan host.docker.internal '
                    'atau 172.17.0.1 jika gagal.')
    kx_arg = DeclareLaunchArgument(
        'Kx', default_value='0.5', description='KBBC gain Kx')
    ky_arg = DeclareLaunchArgument(
        'Ky', default_value='3.0', description='KBBC gain Ky')
    ktheta_arg = DeclareLaunchArgument(
        'Ktheta', default_value='0.5', description='KBBC gain Ktheta')
    kdirect_arg = DeclareLaunchArgument(
        'Kdirect', default_value='0.3', description='KBBC gain Kdirect')

    sim_type  = LaunchConfiguration('sim_type')
    kp        = LaunchConfiguration('kp')
    ki        = LaunchConfiguration('ki')
    kd        = LaunchConfiguration('kd')
    send_host = LaunchConfiguration('send_host')
    Kx        = LaunchConfiguration('Kx')
    Ky        = LaunchConfiguration('Ky')
    Ktheta    = LaunchConfiguration('Ktheta')
    Kdirect   = LaunchConfiguration('Kdirect')

    # Node 1: Master node — seluruh simulasi dalam 1 node
    master_node = Node(
        package='jetbot_control',
        executable='master_node',
        name='master_node',
        parameters=[{
            'fixed_dt':    0.01,
            'start_x':     0.0,
            'start_y':     0.0,
            'start_th':    0.0,
            'listen_port': 5052,
            'send_port':   5053,
            'send_host':   send_host,
            'scale_k':     1.5,
            'omega':       0.1,
            'Kx':          Kx,
            'Ky':          Ky,
            'Ktheta':      Ktheta,
            'Kdirect':     Kdirect,
            'sim_type':    sim_type,
        }],
        output='screen',
    )

    return LaunchDescription([
        sim_type_arg, kp_arg, ki_arg, kd_arg,
        send_host_arg,
        kx_arg, ky_arg, ktheta_arg, kdirect_arg,
        master_node,
    ])
