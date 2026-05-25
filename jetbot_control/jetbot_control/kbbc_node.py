"""
================================================================
kbbc_node.py  —  ROS 2 Node: KBBC Controller  [FIXED]
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
PERBAIKAN [FIX-KBBC-A]:
  MASALAH LAMA: _cb_ref() memeriksa len(msg.data) >= 5, padahal
  trajectory_node mengirim 6 elemen [xr,yr,thr,vr,wr,ELAPSED].
  Akibatnya elapsed (index ke-5) tidak pernah dibaca oleh kbbc_node!
  Fix yang diusulkan di diagnosis sebelumnya sudah ada di trajectory_node
  (publish elapsed di index 5) tapi BELUM diimplementasi di kbbc_node.

  SOLUSI: Ubah pengecekan menjadi >= 6 dan simpan elapsed dari trajectory
  sebagai self.elapsed_traj. Gunakan ini sebagai waktu referensi tunggal
  sehingga semua node sinkron dengan sumber waktu yang sama.

PERBAIKAN [FIX-KBBC-B]:
  MASALAH LAMA: kbbc_node langsung set running=True saat ref_ready dan
  state_ready, tanpa menunggu sistem benar-benar stabil. Saat startup,
  trajectory elapsed sudah maju sehingga error postur besar.

  SOLUSI: kbbc_node mengirim sinyal /system_ready=1 ke trajectory_node
  setelah mendapat KEDUA data (ref_ready AND state_ready). Trajectory_node
  baru mulai naik elapsed setelah menerima sinyal ini. Dengan demikian,
  robot dan referensi selalu mulai dari titik yang sama.

PERBAIKAN [FIX-KBBC-C]:
  MASALAH LAMA: Tidak ada data-fresh check — kbbc_node bisa menghitung
  error dengan data lama (stale) jika satu subscriber terlambat.

  SOLUSI: Tandai ref_fresh dan state_fresh setiap kali data baru tiba.
  Reset flag setelah dipakai. Jika data tidak fresh, skip komputasi.

SUBSCRIBE:
  /reference_state  [Float64MultiArray] — [xr,yr,thr,vr,wr,elapsed]
  /robot_state      [Float64MultiArray] — [x,y,th,v,w]
  /sim_control      [Int32]

PUBLISH:
  /cmd_vel          [Float64MultiArray] — [v_cmd, w_cmd]
  /error_state      [Float64MultiArray] — [ex,ey,eth,vr,wr]
  /wheel_cmd        [Float64MultiArray] — [w_R_req, w_L_req]
  /system_ready     [Int32]             — 1=siap, 0=belum
================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32

from jetbot_control.kbbc_controller import KBBCController
from jetbot_control.robot_plant import R, L


class KBBCNode(Node):

    def __init__(self):
        super().__init__('kbbc_node')

        self.declare_parameter('Kx',      0.5)
        self.declare_parameter('Ky',      3.0)
        self.declare_parameter('Ktheta',  0.5)
        self.declare_parameter('Kdirect', 0.3)
        self.declare_parameter('dt',      0.01)

        Kx      = self.get_parameter('Kx').value
        Ky      = self.get_parameter('Ky').value
        Ktheta  = self.get_parameter('Ktheta').value
        Kdirect = self.get_parameter('Kdirect').value
        self.dt = self.get_parameter('dt').value

        self.kbbc = KBBCController(
            Kx=Kx, Ky=Ky, Ktheta=Ktheta, K_direct=Kdirect)

        # State robot
        self.x  = 0.0; self.y  = 0.0; self.th = 0.0
        self.xr = 0.0; self.yr = 0.0; self.thr = 0.0
        self.vr = 0.0; self.wr = 0.0

        # [FIX-KBBC-A] Simpan elapsed dari trajectory
        self.elapsed_traj = 0.0

        # [FIX-KBBC-B] State sinkronisasi
        self.running      = False
        self.ref_ready    = False
        self.state_ready  = False
        self.ready_sent   = False   # sudah kirim /system_ready?

        # [FIX-KBBC-C] Data freshness flag
        self.ref_fresh    = False
        self.state_fresh  = False

        # Publishers
        self.pub_cmdvel   = self.create_publisher(
            Float64MultiArray, '/cmd_vel', 10)
        self.pub_error    = self.create_publisher(
            Float64MultiArray, '/error_state', 10)
        self.pub_wheelcmd = self.create_publisher(
            Float64MultiArray, '/wheel_cmd', 10)

        # [FIX-KBBC-B] Publisher sinyal system_ready ke trajectory_node
        self.pub_ready = self.create_publisher(
            Int32, '/system_ready', 10)

        # Subscribers
        self.sub_ref = self.create_subscription(
            Float64MultiArray, '/reference_state', self._cb_ref, 10)
        self.sub_state = self.create_subscription(
            Float64MultiArray, '/robot_state', self._cb_state, 10)
        self.sub_ctrl = self.create_subscription(
            Int32, '/sim_control', self._cb_control, 10)

        self.timer = self.create_timer(self.dt, self._timer_cb)

        self.get_logger().info(
            'kbbc_node siap. Kx=%.2f Ky=%.2f Ktheta=%.2f Kdirect=%.2f\n'
            '  [FIX-KBBC-A] Akan baca elapsed dari trajectory (index 5)\n'
            '  [FIX-KBBC-B] Akan kirim /system_ready saat ref + state diterima' %
            (Kx, Ky, Ktheta, Kdirect))

    def _cb_ref(self, msg):
        """Terima state referensi dari trajectory_node."""
        # [FIX-KBBC-A] Baca semua 6 elemen termasuk elapsed di index 5
        if len(msg.data) >= 6:
            self.xr  = msg.data[0]
            self.yr  = msg.data[1]
            self.thr = msg.data[2]
            self.vr  = msg.data[3]
            self.wr  = msg.data[4]
            self.elapsed_traj = msg.data[5]   # ← FIX-KBBC-A: baca elapsed!
            self.ref_ready = True
            self.ref_fresh = True             # ← FIX-KBBC-C

            # [FIX-KBBC-B] Kirim system_ready saat pertama kali kedua data ada
            if self.state_ready and not self.ready_sent:
                self._send_system_ready()

    def _cb_state(self, msg):
        """Terima state robot dari plant_node."""
        if len(msg.data) >= 3:
            self.x   = msg.data[0]
            self.y   = msg.data[1]
            self.th  = msg.data[2]
            self.state_ready = True
            self.state_fresh = True           # ← FIX-KBBC-C

            # [FIX-KBBC-B] Kirim system_ready saat pertama kali kedua data ada
            if self.ref_ready and not self.ready_sent:
                self._send_system_ready()

    def _send_system_ready(self):
        """[FIX-KBBC-B] Beritahu trajectory_node bahwa sistem siap."""
        self.ready_sent = True
        self.running    = True
        msg = Int32()
        msg.data = 1
        self.pub_ready.publish(msg)
        self.get_logger().info(
            '[FIX-KBBC-B] /system_ready dikirim — trajectory akan mulai '
            'menaikkan elapsed. Sistem sinkron!')

    def _cb_control(self, msg):
        cmd = msg.data
        if cmd == 9:
            self.kbbc.reset()
            self.running      = False
            self.ref_ready    = False
            self.state_ready  = False
            self.ready_sent   = False
            self.ref_fresh    = False
            self.state_fresh  = False
            self.elapsed_traj = 0.0
            # Beritahu trajectory untuk reset
            m = Int32(); m.data = 0
            self.pub_ready.publish(m)
            self.get_logger().info('RESET')
        elif cmd == 99:
            self.running = False
            self.get_logger().info('STOP diterima')

    def _timer_cb(self):
        if not (self.running and self.ref_ready and self.state_ready):
            return

        # [FIX-KBBC-C] Skip jika data tidak fresh (salah satu stale)
        if not (self.ref_fresh and self.state_fresh):
            return

        # Reset freshness flag setelah dipakai
        self.ref_fresh   = False
        self.state_fresh = False

        # Hitung error postur (Eq.30)
        ex, ey, eth = self.kbbc.compute_error(
            self.x, self.y, self.th,
            self.xr, self.yr, self.thr)

        # Hitung v_cmd, w_cmd (Eq.31 + FIX-KBBC-1)
        v_cmd, w_cmd = self.kbbc.compute_velocity(
            ex, ey, eth, self.vr, self.wr, self.dt)

        # Inverse kinematics
        w_R_req, w_L_req = self.kbbc.inverse_kinematics(v_cmd, w_cmd, R, L)

        # Publish cmd_vel
        msg_vel = Float64MultiArray()
        msg_vel.data = [v_cmd, w_cmd]
        self.pub_cmdvel.publish(msg_vel)

        # Publish error state (sertakan elapsed_traj untuk sinkronisasi)
        msg_err = Float64MultiArray()
        msg_err.data = [ex, ey, eth, self.vr, self.wr]
        self.pub_error.publish(msg_err)

        # Publish wheel command
        msg_whl = Float64MultiArray()
        msg_whl.data = [w_R_req, w_L_req]
        self.pub_wheelcmd.publish(msg_whl)


def main(args=None):
    rclpy.init(args=args)
    node = KBBCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
