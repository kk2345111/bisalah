"""
================================================================
trajectory_node.py  —  ROS 2 Node: Trajectory Generator  [FIXED]
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
PERBAIKAN [FIX-TRAJ-1]:
  MASALAH LAMA: trajectory_node langsung naik elapsed sejak
  pertama kali timer dipanggil, bahkan sebelum plant_node dan
  kbbc_node siap. Akibatnya, saat sistem baru startup, elapsed
  sudah jauh lebih besar dari 0, sehingga error postur langsung
  besar dan robot bergerak tidak stabil.

  SOLUSI: trajectory_node menunggu sinyal /system_ready (Int32=1)
  dari kbbc_node sebelum mulai menaikkan elapsed. Selama menunggu,
  node tetap publish state awal (t=0) agar kbbc_node punya data
  referensi yang valid.

PERBAIKAN [FIX-TRAJ-2]:
  Format publish /reference_state sudah 6 elemen [xr,yr,thr,vr,wr,elapsed].
  Pastikan elapsed yang dipublish = elapsed simulation yang sebenarnya.

PUBLISH:
  /reference_state  [Float64MultiArray]
    data: [xr, yr, thr, vr, wr, elapsed]   (6 elemen)

SUBSCRIBE:
  /sim_control    [Int32]  — 0=jalan, 9=RESET, 99=STOP
  /system_ready   [Int32]  — 1=sistem siap, mulai naik elapsed

PARAMETER:
  scale_k : float = 1.5
  omega   : float = 0.1
  dt      : float = 0.01
================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32

from jetbot_control.trajectory_generator import TrajectoryGenerator


class TrajectoryNode(Node):

    def __init__(self):
        super().__init__('trajectory_node')

        self.declare_parameter('scale_k', 1.5)
        self.declare_parameter('omega',   0.1)
        self.declare_parameter('dt',      0.01)

        scale_k  = self.get_parameter('scale_k').value
        omega    = self.get_parameter('omega').value
        self.dt  = self.get_parameter('dt').value

        self.traj    = TrajectoryGenerator(scale=scale_k, omega=omega)
        self.elapsed = 0.0

        # [FIX-TRAJ-1] Tunggu sinyal system_ready sebelum naik elapsed
        self.running      = False   # elapsed belum naik
        self.system_ready = False   # kbbc siap?

        self.pub_ref = self.create_publisher(
            Float64MultiArray, '/reference_state', 10)

        self.sub_ctrl = self.create_subscription(
            Int32, '/sim_control', self._cb_control, 10)

        # [FIX-TRAJ-1] Subscribe sinyal READY dari kbbc_node
        self.sub_ready = self.create_subscription(
            Int32, '/system_ready', self._cb_ready, 10)

        self.timer = self.create_timer(self.dt, self._timer_cb)

        self.get_logger().info(
            'trajectory_node siap. k=%.2f m  omega=%.3f rad/s  T=%.1f s\n'
            '  Menunggu /system_ready sebelum memulai elapsed...' %
            (scale_k, omega, self.traj.period))

    def _cb_ready(self, msg):
        """Terima sinyal system_ready dari kbbc_node."""
        if msg.data == 1 and not self.system_ready:
            self.system_ready = True
            self.running      = True
            self.get_logger().info(
                '[FIX-TRAJ-1] System READY — elapsed mulai naik dari %.3f s' %
                self.elapsed)

    def _cb_control(self, msg):
        cmd = msg.data
        if cmd == 9:
            self.elapsed      = 0.0
            self.running      = False
            self.system_ready = False
            self.get_logger().info(
                'RESET — elapsed kembali ke 0, tunggu system_ready lagi')
        elif cmd == 99:
            self.running = False
            self.get_logger().info('STOP diterima')

    def _timer_cb(self):
        # [FIX-TRAJ-2] Selalu publish, tapi elapsed hanya naik saat running
        if self.running:
            self.elapsed += self.dt

        xr, yr, thr, vr, wr = self.traj.get_state(self.elapsed)

        msg = Float64MultiArray()
        msg.data = [xr, yr, thr, vr, wr, self.elapsed]
        self.pub_ref.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
