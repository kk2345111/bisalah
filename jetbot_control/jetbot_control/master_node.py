"""
================================================================
master_node.py  —  MASTER SIMULATION NODE (Single Node)
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
FILOSOFI DESAIN:
  Masalah utama arsitektur multi-node (trajectory_node, plant_node,
  kbbc_node, pid_node, dll.) adalah ketidaksinkronan antar node
  akibat CPU terbatas (QEMU ARM64). Setiap node punya timer 10ms
  independen yang bisa drift, dan komunikasi antar node via topic
  menambah latency dan packet drop.

  SOLUSI: Gabungkan semua logika ke SATU node dengan SATU timer
  callback, persis seperti main_simulation.py di Windows. Tidak
  ada komunikasi antar node — semua variabel shared langsung dalam
  satu thread.

KEUNGGULAN vs ARSITEKTUR MULTI-NODE:
  1. TIDAK ada masalah sinkronisasi — trajectory, KBBC, PID, plant
     semua berjalan dalam satu callback yang sama, urutan deterministik
  2. TIDAK ada startup race condition — semua modul diinisialisasi
     bersamaan dalam __init__()
  3. CPU lebih ringan — 1 timer callback vs 7 timer callback (7x lebih
     sedikit overhead Python/ROS2 per detik)
  4. dt SELALU 0.01s — tidak ada jitter karena tidak ada antar-node
     komunikasi yang bisa memperlambat
  5. Sinkron dengan main_simulation.py — hasil simulasi seharusnya
     identik dengan versi Windows

STRUKTUR CALLBACK SATU ITERASI (urutan sama dengan main_simulation.py):
  1. Baca parameter dari LabVIEW (non-blocking)
  2. Handle RESET / STOP / mode change
  3. Naikkan elapsed
  4. Hitung referensi trajektori (Eq.40)
  5. Hitung error postur (Eq.30) + KBBC (Eq.31)
  6. Inverse kinematics → kecepatan roda
  7. Motor controller (KBBC Only atau KBBC+PID)
  8. Fisika Lagrangian RK4
  9. Encoder + odometri
  10. Kirim UDP ke LabVIEW
  11. Log CSV

PUBLISH topic (untuk monitoring via ros2 topic echo):
  /master/robot_state   [Float64MultiArray] — [x,y,th,v,w,elapsed]
  /master/error_state   [Float64MultiArray] — [ex,ey,eth,vr,wr]
  /master/motor_voltage [Float64MultiArray] — [volt_L, volt_R]
  /master/debug_info    [String]            — ringkasan per detik

SUBSCRIBE:
  /sim_control          [Int32]  — 9=RESET, 99=STOP (dari LabVIEW bridge
                                   atau manual)

UDP RECEIVE (dari LabVIEW, format sama dengan main_simulation.py):
  8 × double big-endian (64 byte):
  [0]kp_R [1]ki_R [2]kd_R [3]kp_L [4]ki_L [5]kd_L [6]reset [7]sim_type

UDP SEND (ke LabVIEW, CSV 14 nilai):
  x,y,theta,v,w,ex,ey,eth,pwmL,pwmR,vr,tauL,tauR,sim_type
================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Int32

import socket
import struct
import math
import csv
import time

from jetbot_control.trajectory_generator import TrajectoryGenerator
from jetbot_control.robot_plant import (
    RobotPlant, R, L, MAX_VOLT, K_b, N_g
)
from jetbot_control.kbbc_controller import KBBCController
from jetbot_control.pid_controller import (
    MotorPIDController, scale_volt, I_BIAS, R_a
)
from jetbot_control.sensor_model import EncoderModel, OdometryModel


# ── Konfigurasi default ──────────────────────────────────────────
FIXED_DT     = 0.01       # 10ms control loop
START_X      = 0.0
START_Y      = 0.0
START_TH     = 0.0
LISTEN_PORT  = 5052       # Terima parameter dari LabVIEW
SEND_PORT    = 5053       # Kirim data ke LabVIEW
SEND_HOST    = 'host.docker.internal'  # IP host Windows dari Docker
DEBUG_EVERY  = 100        # Print debug setiap N iterasi (100 × 10ms = 1 detik)
LOG_DIR      = '/ta_jetbot_ws'


class MasterNode(Node):
    """
    Single ROS2 node yang menjalankan seluruh simulasi dalam
    satu timer callback 10ms. Tidak ada dependensi node lain.
    """

    def __init__(self):
        super().__init__('master_node')

        # ── Declare parameters ───────────────────────────────────
        self.declare_parameter('fixed_dt',   FIXED_DT)
        self.declare_parameter('start_x',    START_X)
        self.declare_parameter('start_y',    START_Y)
        self.declare_parameter('start_th',   START_TH)
        self.declare_parameter('listen_port', LISTEN_PORT)
        self.declare_parameter('send_port',   SEND_PORT)
        self.declare_parameter('send_host',   SEND_HOST)
        self.declare_parameter('scale_k',    1.5)
        self.declare_parameter('omega',      0.1)
        self.declare_parameter('Kx',         0.5)
        self.declare_parameter('Ky',         3.0)
        self.declare_parameter('Ktheta',     0.5)
        self.declare_parameter('Kdirect',    0.3)
        self.declare_parameter('sim_type',   0)

        self.dt       = self.get_parameter('fixed_dt').value
        self.x0       = self.get_parameter('start_x').value
        self.y0       = self.get_parameter('start_y').value
        self.th0      = self.get_parameter('start_th').value
        lport         = self.get_parameter('listen_port').value
        sport         = self.get_parameter('send_port').value
        send_host     = self.get_parameter('send_host').value
        scale_k       = self.get_parameter('scale_k').value
        omega         = self.get_parameter('omega').value
        Kx            = self.get_parameter('Kx').value
        Ky            = self.get_parameter('Ky').value
        Ktheta        = self.get_parameter('Ktheta').value
        Kdirect       = self.get_parameter('Kdirect').value
        self.sim_type = self.get_parameter('sim_type').value

        # ── Resolve IP send host ─────────────────────────────────
        try:
            send_ip = socket.gethostbyname(send_host)
        except socket.gaierror:
            send_ip = '172.17.0.1'
            self.get_logger().warn(
                'Gagal resolve %s, fallback ke %s' % (send_host, send_ip))
        self._labview_addr = (send_ip, sport)

        # ── UDP socket (non-blocking) ────────────────────────────
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('0.0.0.0', lport))
        self._sock.settimeout(0)   # NON-BLOCKING — tidak ada sleep di callback!

        # ── Inisialisasi semua modul simulasi ────────────────────
        self._traj  = TrajectoryGenerator(scale=scale_k, omega=omega)
        self._kbbc  = KBBCController(
            Kx=Kx, Ky=Ky, Ktheta=Ktheta, K_direct=Kdirect)
        self._pid_R = MotorPIDController(side='R')
        self._pid_L = MotorPIDController(side='L')
        self._enc_R = EncoderModel(ppr=20, side='R')
        self._enc_L = EncoderModel(ppr=20, side='L')
        self._odom  = OdometryModel(self.x0, self.y0, self.th0)
        self._robot = RobotPlant(self.x0, self.y0, self.th0)

        # ── State simulasi ───────────────────────────────────────
        self.elapsed    = 0.0
        self._iter      = 0
        self._kp_R      = 0.0;  self._ki_R = 0.0;  self._kd_R = 0.0
        self._kp_L      = 0.0;  self._ki_L = 0.0;  self._kd_L = 0.0
        self._prev_mode = -1
        self._running   = True

        # State terakhir untuk publish
        self._last = {
            'x': 0.0, 'y': 0.0, 'th': 0.0, 'v': 0.0, 'w': 0.0,
            'ex': 0.0, 'ey': 0.0, 'eth': 0.0,
            'vr': 0.0, 'wr': 0.0,
            'volt_L': 0.0, 'volt_R': 0.0,
            'tau_L': 0.0, 'tau_R': 0.0,
        }

        # ── Publishers (untuk monitoring) ────────────────────────
        self._pub_state = self.create_publisher(
            Float64MultiArray, '/master/robot_state', 10)
        self._pub_err = self.create_publisher(
            Float64MultiArray, '/master/error_state', 10)
        self._pub_volt = self.create_publisher(
            Float64MultiArray, '/master/motor_voltage', 10)
        self._pub_debug = self.create_publisher(
            String, '/master/debug_info', 10)

        # ── Subscriber (untuk RESET/STOP manual) ────────────────
        self.create_subscription(
            Int32, '/sim_control', self._cb_ctrl, 10)

        # ── CSV Logger ───────────────────────────────────────────
        ts = int(time.time())
        self._fname = '%s/LOG_JETBOT_%d.csv' % (LOG_DIR, ts)
        self._fcsv  = open(self._fname, 'w', newline='')
        self._cw    = csv.writer(self._fcsv)
        self._cw.writerow([
            'Time',
            'X_true', 'Y_true', 'Theta_true',
            'X_est',  'Y_est',  'Theta_est',
            'V_rob',  'W_rob',
            'Ex', 'Ey', 'Eth',
            'Vr', 'Wr',
            'Volt_L',    'Volt_R',
            'Tau_L',     'Tau_R',
            'iR',        'iL',
            'OmR_plant', 'OmL_plant',
            'OmR_enc',   'OmL_enc',
            'Drift_m',
            'SimType',
        ])

        # ── SATU timer callback — ini jantung dari master_node ───
        self._timer = self.create_timer(self.dt, self._main_loop)

        self.get_logger().info (
            (
            '\n' + '=' * 60 + '\n'
            '  MASTER NODE — Simulasi JetBot (Single Node)\n'
            '  Trajectory: k=%.2f  omega=%.3f  T=%.1f s\n'
            '  KBBC: Kx=%.2f  Ky=%.2f  Ktheta=%.2f  Kdirect=%.2f\n'
            '  UDP listen: 0.0.0.0:%d\n'
            '  UDP send  : %s:%d\n'
            '  Log CSV   : %s\n'
            '  sim_type  : %d (0=KBBC Only, 1=KBBC+PID)\n'
            + '=' * 60 ) % (
                scale_k, omega, self._traj.period,
                Kx, Ky, Ktheta, Kdirect,
                lport,
                send_ip, sport,
                self._fname,
                self.sim_type,
            )
        )

    # ── Subscriber callback untuk RESET/STOP manual ──────────────
    def _cb_ctrl(self, msg):
        if msg.data == 9:
            self._do_reset()
        elif msg.data == 99:
            self._running = False
            self.get_logger().info('STOP via /sim_control')

    def _do_reset(self):
        """Reset semua state simulasi ke posisi awal."""
        self._robot.reset(self.x0, self.y0, self.th0)
        self._kbbc.reset()
        self._pid_R.reset()
        self._pid_L.reset()
        self._enc_R.reset()
        self._enc_L.reset()
        self._odom.reset(self.x0, self.y0, self.th0)
        self.elapsed    = 0.0
        self._iter      = 0
        self._prev_mode = -1
        self.get_logger().info(
            'RESET — semua state kembali ke awal. '
            'Mode: %s' % ('KBBC+PID' if self.sim_type == 1 else 'KBBC Only'))

    # ════════════════════════════════════════════════════════════
    # MAIN LOOP — dipanggil tepat setiap FIXED_DT = 10ms
    # Urutan identik dengan main_simulation.py
    # ════════════════════════════════════════════════════════════
    def _main_loop(self):
        if not self._running:
            return

        self._iter += 1

        # ── STEP 1: Baca UDP dari LabVIEW (non-blocking) ─────────
        try:
            data, addr = self._sock.recvfrom(1024)
            if len(data) >= 64:
                u = struct.unpack('>dddddddd', data[0:64])
                kp_R_new     = u[0];  ki_R_new = u[1];  kd_R_new = u[2]
                kp_L_new     = u[3];  ki_L_new = u[4];  kd_L_new = u[5]
                mode_reset   = int(u[6])
                sim_type_new = int(u[7])

                self._kp_R = kp_R_new;  self._ki_R = ki_R_new
                self._kd_R = kd_R_new
                self._kp_L = kp_L_new;  self._ki_L = ki_L_new
                self._kd_L = kd_L_new
                self._pid_R.update_gains(kp_R_new, ki_R_new, kd_R_new)
                self._pid_L.update_gains(kp_L_new, ki_L_new, kd_L_new)

                # Handle STOP
                if mode_reset == 99:
                    self._running = False
                    self.get_logger().info('STOP dari LabVIEW')
                    return

                # Handle RESET
                if mode_reset == 9:
                    self.sim_type = sim_type_new
                    self._do_reset()
                    return

                # Update sim_type
                self.sim_type = sim_type_new

                # Simpan addr untuk kirim balik
                self._labview_addr = (addr[0], self._labview_addr[1])

        except BlockingIOError:
            pass    # Tidak ada data UDP — normal, lanjut
        except Exception as e:
            self.get_logger().warn('UDP RX error: %s' % str(e))

        # ── STEP 2: Deteksi ganti mode ────────────────────────────
        if self.sim_type != self._prev_mode:
            self._kbbc.reset()
            self._pid_R.reset()
            self._pid_L.reset()
            self._prev_mode = self.sim_type
            self.get_logger().info(
                'Mode berganti → %s' %
                ('KBBC+PID' if self.sim_type == 1 else 'KBBC Only'))

        # ── STEP 3: Naikkan elapsed ───────────────────────────────
        self.elapsed += self.dt

        # ── STEP 4: Trajektori referensi (Eq.40) ─────────────────
        xr, yr, thr, vr, wr = self._traj.get_state(self.elapsed)

        # ── STEP 5: Error postur (Eq.30) + KBBC (Eq.31) ──────────
        xf, yf, thf = self._robot.pos
        ex, ey, eth = self._kbbc.compute_error(xf, yf, thf, xr, yr, thr)
        v_cmd, w_cmd = self._kbbc.compute_velocity(
            ex, ey, eth, vr, wr, self.dt)

        # ── STEP 6: Inverse kinematics ────────────────────────────
        w_R_req, w_L_req = self._kbbc.inverse_kinematics(
            v_cmd, w_cmd, R, L)

        # ── STEP 7: Motor controller ──────────────────────────────
        #    Feedback kecepatan roda dari plant langsung (sesuai jurnal)
        w_R_act, w_L_act = self._robot.wheel_speeds
        volt_R = self._pid_R.compute(
            w_R_req, w_R_act, self.sim_type, self.dt)
        volt_L = self._pid_L.compute(
            w_L_req, w_L_act, self.sim_type, self.dt)

        # Scaling proporsional saat saturasi [FIX-M1]
        volt_R, volt_L = scale_volt(volt_R, volt_L, MAX_VOLT)

        # ── STEP 8: Fisika Lagrangian RK4 ────────────────────────
        vc, wc, xc, yc, thc, tau_R, tau_L = self._robot.step(
            volt_L, volt_R, self.dt)

        # ── STEP 9: Encoder + odometri ───────────────────────────
        omRp, omLp = self._robot.wheel_speeds
        omRe = self._enc_R.update(omRp, volt_R, self.dt)
        omLe = self._enc_L.update(omLp, volt_L, self.dt)
        xe, ye, the, _, _ = self._odom.update(omRe, omLe, self.dt)
        drm = self._odom.drift(xc, yc, thc)

        # ── STEP 10: Kirim UDP ke LabVIEW ────────────────────────
        pwL = volt_L * (100.0 / MAX_VOLT)
        pwR = volt_R * (100.0 / MAX_VOLT)
        msg_udp = (
            '%.3f,%.3f,%.3f,'     # x, y, theta
            '%.3f,%.3f,'          # v, w
            '%.3f,%.3f,%.3f,'     # ex, ey, eth
            '%.3f,%.3f,'          # pwmL, pwmR
            '%.3f,%.4f,%.4f,'     # vr, tau_L, tau_R
            '%d'                  # sim_type  (14 nilai total)
        ) % (
            xc, yc, thc,
            vc, wc,
            ex, ey, eth,
            pwL, pwR,
            vr, tau_L, tau_R,
            self.sim_type,
        )
        try:
            self._sock.sendto(msg_udp.encode(), self._labview_addr)
        except Exception:
            pass

        # ── STEP 11: Log CSV ──────────────────────────────────────
        self._cw.writerow([
            '%.3f' % self.elapsed,
            '%.4f' % xc,  '%.4f' % yc,  '%.4f' % thc,
            '%.4f' % xe,  '%.4f' % ye,  '%.4f' % the,
            '%.4f' % vc,  '%.4f' % wc,
            '%.4f' % ex,  '%.4f' % ey,  '%.4f' % eth,
            '%.4f' % vr,  '%.4f' % wr,
            '%.3f' % volt_L, '%.3f' % volt_R,
            '%.5f' % tau_L,  '%.5f' % tau_R,
            '%.4f' % self._robot.i_R, '%.4f' % self._robot.i_L,
            '%.3f' % omRp, '%.3f' % omLp,
            '%.3f' % omRe, '%.3f' % omLe,
            '%.4f' % drm,
            '%d' % self.sim_type,
        ])

        # Flush setiap 100 baris agar tidak hilang saat crash
        if self._iter % 100 == 0:
            self._fcsv.flush()

        # ── STEP 12: Publish ke topic ROS2 (untuk monitoring) ────
        msg_state = Float64MultiArray()
        msg_state.data = [xc, yc, thc, vc, wc, self.elapsed]
        self._pub_state.publish(msg_state)

        msg_err = Float64MultiArray()
        msg_err.data = [ex, ey, eth, vr, wr]
        self._pub_err.publish(msg_err)

        msg_volt = Float64MultiArray()
        msg_volt.data = [volt_L, volt_R]
        self._pub_volt.publish(msg_volt)

        # ── STEP 13: Debug print setiap 1 detik ──────────────────
        if self._iter % DEBUG_EVERY == 0:
            mode_str = 'KBBC+PID' if self.sim_type == 1 else 'KBBC Only'
            debug = (
                't=%.2fs | %s | pos=(%.3f,%.3f,%.1fdeg) | '
                'ref=(%.3f,%.3f) | err=(%.3f,%.3f,%.1fdeg) | '
                'v=%.3f vr=%.3f | VL=%.2f VR=%.2f'
            ) % (
                self.elapsed, mode_str,
                xc, yc, math.degrees(thc),
                xr, yr,
                ex, ey, math.degrees(eth),
                vc, vr,
                volt_L, volt_R,
            )
            self.get_logger().info(debug)
            msg_dbg = String()
            msg_dbg.data = debug
            self._pub_debug.publish(msg_dbg)

    def destroy_node(self):
        """Cleanup saat node dihentikan."""
        try:
            self._fcsv.flush()
            self._fcsv.close()
        except Exception:
            pass
        try:
            self._sock.close()
        except Exception:
            pass
        self.get_logger().info(
            'Master node selesai. Log: %s  Durasi: %.2fs  '
            'Drift: %.4fm' % (
                self._fname, self.elapsed, self._odom.drift_accum))
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
