"""
================================================================
pid_node.py  —  ROS 2 Node: PID Motor Controller  [FIXED]
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
PERBAIKAN [FIX-PID-N1]:
  MASALAH LAMA: _timer_cb() selalu publish [0.0, 0.0] saat
  not running. Ini menyebabkan plant_node menerima /motor_voltage
  terus-menerus dan robot_plant.step() dipanggil setiap 10ms
  bahkan saat sistem belum siap. Boros CPU, dan (lebih penting)
  menyebabkan plant_node langsung running=True padahal volt=0.

  SOLUSI: Jangan publish saat not running. plant_node akan tetap
  publish robot_state (posisi awal) tanpa perlu dapat voltase.
  Dengan [FIX-PLANT-N1], deadlock startup sudah teratasi.

PERBAIKAN [FIX-PID-N2]:
  Reset PID integral saat mode berganti (sim_type berubah).
  Ini mencegah integral windup yang terakumulasi saat idle
  langsung mempengaruhi output saat mode pertama kali aktif.

SUBSCRIBE:
  /wheel_cmd          [Float64MultiArray] — [w_R_req, w_L_req]
  /wheel_speed_plant  [Float64MultiArray] — [w_R_act, w_L_act]
  /labview_params     [Float64MultiArray] — [kpR,kiR,kdR,kpL,kiL,kdL,reset,type]
  /sim_control        [Int32]

PUBLISH:
  /motor_voltage      [Float64MultiArray] — [volt_L, volt_R]
================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32

from jetbot_control.pid_controller import MotorPIDController, scale_volt
from jetbot_control.robot_plant import MAX_VOLT


class PIDNode(Node):

    def __init__(self):
        super().__init__('pid_node')

        self.declare_parameter('kp_R',    0.1)
        self.declare_parameter('ki_R',    0.02)
        self.declare_parameter('kd_R',    0.005)
        self.declare_parameter('kp_L',    0.1)
        self.declare_parameter('ki_L',    0.02)
        self.declare_parameter('kd_L',    0.005)
        self.declare_parameter('sim_type', 0)
        self.declare_parameter('dt',       0.01)

        kp_R = self.get_parameter('kp_R').value
        ki_R = self.get_parameter('ki_R').value
        kd_R = self.get_parameter('kd_R').value
        kp_L = self.get_parameter('kp_L').value
        ki_L = self.get_parameter('ki_L').value
        kd_L = self.get_parameter('kd_L').value
        self.sim_type     = self.get_parameter('sim_type').value
        self.dt           = self.get_parameter('dt').value
        self._prev_sim_type = self.sim_type

        self.pid_R = MotorPIDController(Kp=kp_R, Ki=ki_R, Kd=kd_R, side='R')
        self.pid_L = MotorPIDController(Kp=kp_L, Ki=ki_L, Kd=kd_L, side='L')

        self.w_R_req  = 0.0;  self.w_L_req  = 0.0
        self.w_R_act  = 0.0;  self.w_L_act  = 0.0
        self.running   = False
        self.cmd_ready = False

        self.pub_volt = self.create_publisher(
            Float64MultiArray, '/motor_voltage', 10)

        self.sub_cmd = self.create_subscription(
            Float64MultiArray, '/wheel_cmd',
            self._cb_wheelcmd, 10)
        self.sub_act = self.create_subscription(
            Float64MultiArray, '/wheel_speed_plant',
            self._cb_wheelact, 10)
        self.sub_lv = self.create_subscription(
            Float64MultiArray, '/labview_params',
            self._cb_labview, 10)
        self.sub_ctrl = self.create_subscription(
            Int32, '/sim_control', self._cb_control, 10)

        self.timer = self.create_timer(self.dt, self._timer_cb)

        self.get_logger().info(
            'pid_node siap [FIX-PID-N1,N2]. sim_type=%d '
            '(0=KBBC Only, 1=KBBC+PID)\n'
            '  Tidak akan publish volt=0 saat tidak running.' %
            self.sim_type)

    def _cb_wheelcmd(self, msg):
        if len(msg.data) >= 2:
            self.w_R_req  = msg.data[0]
            self.w_L_req  = msg.data[1]
            self.cmd_ready = True
            if not self.running:
                self.running = True

    def _cb_wheelact(self, msg):
        if len(msg.data) >= 2:
            self.w_R_act = msg.data[0]
            self.w_L_act = msg.data[1]

    def _cb_labview(self, msg):
        if len(msg.data) >= 8:
            kp_R = msg.data[0];  ki_R = msg.data[1];  kd_R = msg.data[2]
            kp_L = msg.data[3];  ki_L = msg.data[4];  kd_L = msg.data[5]
            mode_reset = int(msg.data[6])
            sim_type   = int(msg.data[7])

            self.pid_R.update_gains(kp_R, ki_R, kd_R)
            self.pid_L.update_gains(kp_L, ki_L, kd_L)

            # [FIX-PID-N2] Reset integral saat mode berganti
            if sim_type != self._prev_sim_type:
                self.pid_R.reset()
                self.pid_L.reset()
                self._prev_sim_type = sim_type
                self.get_logger().info(
                    '[FIX-PID-N2] Mode berganti %d→%d, PID integral di-reset' %
                    (self._prev_sim_type, sim_type))

            self.sim_type = sim_type

            mode_str = 'KBBC+PID' if sim_type == 1 else 'KBBC Only'
            self.get_logger().info(
                '[LabVIEW] mode=%s | '
                'KpR=%.4f KiR=%.4f KdR=%.4f | '
                'KpL=%.4f KiL=%.4f KdL=%.4f' %
                (mode_str, kp_R, ki_R, kd_R, kp_L, ki_L, kd_L))

    def _cb_control(self, msg):
        cmd = msg.data
        if cmd == 9:
            self.pid_R.reset()
            self.pid_L.reset()
            self.w_R_req  = 0.0;  self.w_L_req  = 0.0
            self.w_R_act  = 0.0;  self.w_L_act  = 0.0
            self.running   = False
            self.cmd_ready = False
            self.get_logger().info('RESET — PID integral di-nol-kan')
        elif cmd == 99:
            self.running = False
            self.get_logger().info('STOP diterima')

    def _timer_cb(self):
        # [FIX-PID-N1] Jangan publish sama sekali saat tidak running
        # Ini mencegah plant_node "terpaksa" running hanya karena terima volt=0
        if not (self.running and self.cmd_ready):
            return

        volt_R = self.pid_R.compute(
            self.w_R_req, self.w_R_act, self.sim_type, self.dt)
        volt_L = self.pid_L.compute(
            self.w_L_req, self.w_L_act, self.sim_type, self.dt)

        # Scaling proporsional saat saturasi [FIX-M1]
        volt_R, volt_L = scale_volt(volt_R, volt_L, MAX_VOLT)

        msg = Float64MultiArray()
        msg.data = [volt_L, volt_R]
        self.pub_volt.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
