"""
================================================================
sensor_node.py  —  ROS 2 Node: Encoder + Odometry
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
FUNGSI:
  Mensimulasikan sensor encoder MH-Sensor-Series dan menghitung
  estimasi posisi robot via dead reckoning (odometri).

SUBSCRIBE:
  /wheel_speed_plant  [Float64MultiArray]  dari plant_node
    data: [omR_plant, omL_plant]  (rad/s, ground truth)

  /motor_voltage      [Float64MultiArray]  dari pid_node
    data: [volt_L, volt_R]  (untuk menentukan arah encoder)

  /robot_state        [Float64MultiArray]  dari plant_node
    data: [x, y, theta, v, w]  (untuk hitung drift)

  /sim_control        [Int32]
    9=reset, 99=stop

PUBLISH:
  /odom_est           [Float64MultiArray]
    data: [x_est, y_est, th_est, v_est, w_est,
           omR_enc, omL_enc, drift_m]
    index:   0       1       2       3     4
               5        6       7

CATATAN PENTING — PERAN SENSOR DI SIMULASI:
  Encoder HANYA untuk odometri (estimasi posisi X_est, Y_est).
  Feedback kecepatan ke PID diambil dari /wheel_speed_plant
  (plant langsung), sesuai struktur Simulink jurnal.

  Dead zone encoder 84-85% adalah NORMAL untuk MH-Sensor-Series
  20 lubang karena resolusinya rendah (18 deg per pulsa).
================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32

from jetbot_control.sensor_model import EncoderModel, OdometryModel


class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor_node')

        self.declare_parameter('dt', 0.01)
        self.dt = self.get_parameter('dt').value

        # ── Inisialisasi sensor ──────────────────────────────
        self.enc_R = EncoderModel(ppr=20, side='R')
        self.enc_L = EncoderModel(ppr=20, side='L')
        self.odom  = OdometryModel(0.0, 0.0, 0.0)

        # State
        self.omR_plant = 0.0;  self.omL_plant = 0.0
        self.volt_L    = 0.0;  self.volt_R    = 0.0
        self.x_true    = 0.0;  self.y_true    = 0.0;  self.th_true = 0.0
        self.running   = False

        # ── Publisher ────────────────────────────────────────
        self.pub_odom = self.create_publisher(
            Float64MultiArray, '/odom_est', 10)

        # ── Subscribers ──────────────────────────────────────
        self.sub_wheels = self.create_subscription(
            Float64MultiArray, '/wheel_speed_plant',
            self._cb_wheels, 10)
        self.sub_volt = self.create_subscription(
            Float64MultiArray, '/motor_voltage',
            self._cb_volt, 10)
        self.sub_state = self.create_subscription(
            Float64MultiArray, '/robot_state',
            self._cb_state, 10)
        self.sub_ctrl = self.create_subscription(
            Int32, '/sim_control', self._cb_control, 10)

        # ── Timer ────────────────────────────────────────────
        self.timer = self.create_timer(self.dt, self._timer_cb)

        self.get_logger().info(
            'sensor_node siap. '
            'Encoder: 20 PPR, window=300ms')

    def _cb_wheels(self, msg):
        if len(msg.data) >= 2:
            self.omR_plant = msg.data[0]
            self.omL_plant = msg.data[1]
            self.running   = True

    def _cb_volt(self, msg):
        if len(msg.data) >= 2:
            self.volt_L = msg.data[0]
            self.volt_R = msg.data[1]

    def _cb_state(self, msg):
        if len(msg.data) >= 3:
            self.x_true  = msg.data[0]
            self.y_true  = msg.data[1]
            self.th_true = msg.data[2]

    def _cb_control(self, msg):
        cmd = msg.data
        if cmd == 9:
            self.enc_R.reset()
            self.enc_L.reset()
            self.odom.reset(0.0, 0.0, 0.0)
            self.running = False
            self.get_logger().info('RESET — encoder dan odometri di-reset')
        elif cmd == 99:
            self.running = False
            self.get_logger().info('STOP diterima')

    def _timer_cb(self):
        if not self.running:
            return

        # Update encoder dari kecepatan plant (bukan dari hardware)
        omR_enc = self.enc_R.update(self.omR_plant, self.volt_R, self.dt)
        omL_enc = self.enc_L.update(self.omL_plant, self.volt_L, self.dt)

        # Update odometri dari encoder
        xe, ye, the, ve, we = self.odom.update(omR_enc, omL_enc, self.dt)

        # Hitung drift vs ground truth
        drift = self.odom.drift(self.x_true, self.y_true, self.th_true)

        msg = Float64MultiArray()
        msg.data = [xe, ye, the, ve, we, omR_enc, omL_enc, drift]
        self.pub_odom.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
