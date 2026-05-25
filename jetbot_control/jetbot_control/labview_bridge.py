"""
================================================================
labview_bridge.py  —  ROS 2 Node: LabVIEW UDP Bridge  [FIXED]
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
PERBAIKAN [FIX-BRIDGE-1] — IP tujuan send ke LabVIEW:
  MASALAH LAMA: labview_addr diset dari addr[0] (IP pengirim paket
  UDP yang diterima). Di Docker Desktop Windows, addr[0] adalah IP
  internal Docker NAT (bisa 192.168.65.x atau 172.17.x.x), BUKAN
  IP host Windows. Akibatnya, data CSV tidak pernah sampai ke LabVIEW!

  SOLUSI: Tambah parameter send_host. Default: 'host.docker.internal'
  yang secara otomatis resolve ke IP host Windows dari dalam Docker.
  Fallback: '172.17.0.1' (gateway Docker default).
  Jika LabVIEW di Windows pakai 127.0.0.1, set send_host=host.docker.internal.

PERBAIKAN [FIX-BRIDGE-2] — Frekuensi kirim ke LabVIEW:
  MASALAH LAMA: timer_send = 0.1 detik (10Hz). Simulasi jalan 100Hz.
  LabVIEW hanya terima 1 dari 10 frame data. Grafik sangat kasar
  dan tidak bisa dipakai untuk analisis presisi.

  SOLUSI: timer_send = 0.01 detik (100Hz). Cocok dengan frekuensi
  simulasi, LabVIEW bisa terima setiap frame data.
  CATATAN: LabVIEW Timed Loop harus diset ke 10ms agar sinkron.

PERBAIKAN [FIX-BRIDGE-3] — Logging UDP yang terlalu sering:
  MASALAH LAMA: get_logger().info() dipanggil setiap paket UDP
  diterima dari LabVIEW. Jika LabVIEW kirim setiap 10ms = 100
  log/detik! Setiap log = 7 baris string → sangat boros CPU.
  Di ARM64 QEMU, ini bisa menyumbang 200-400% CPU sendiri.

  SOLUSI: Log hanya setiap 100 paket (±1 detik sekali). Tetap
  tampilkan info penting (mode berganti, RESET, gains) tapi
  kurangi spam log drastis.

PERBAIKAN [FIX-BRIDGE-4] — Format CSV 15 nilai vs LabVIEW 14:
  MASALAH LAMA: labview_bridge kirim 15 nilai (ada 'drift' di akhir)
  sementara LabVIEW Scan From String pakai 14 format specifier.
  Meski LabVIEW toleran, ini inkonsistensi yang bisa menyebabkan
  issue jika format LabVIEW diubah.

  SOLUSI: Format CSV disesuaikan menjadi 14 nilai (hapus drift
  dari CSV utama ke LabVIEW). Drift tetap tersimpan di CSV logger.
  Format akhir: x,y,theta,v,w,ex,ey,eth,pwmL,pwmR,vr,tauL,tauR,sim_type

ALUR KOMUNIKASI:
  LabVIEW (Windows, port 5052)
      │ UDP 64 byte (8 × double big-endian)
      ▼
  labview_bridge (Docker, listen 0.0.0.0:5052)
      │ /labview_params, /sim_control
      ▼
  pid_node, kbbc_node, semua node

  semua node → /robot_state, /error_state, /motor_voltage, /motor_state
      │
  labview_bridge
      │ UDP CSV string (14 nilai) @ 100Hz [FIX-BRIDGE-2]
      ▼
  LabVIEW (Windows, listen port 5053)

FORMAT UDP TERIMA (dari LabVIEW):
  [0] kp_R   [1] ki_R   [2] kd_R
  [3] kp_L   [4] ki_L   [5] kd_L
  [6] mode_reset  (9=RESET, 99=STOP)
  [7] sim_type    (0=KBBC Only, 1=KBBC+PID)

FORMAT UDP KIRIM (ke LabVIEW, 14 nilai CSV):
  x,y,theta,v,w,ex,ey,eth,pwmL,pwmR,vr,tauL,tauR,sim_type
================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
import socket
import struct
import threading

# Konfigurasi port
LISTEN_IP   = '0.0.0.0'
LISTEN_PORT = 5052
SEND_PORT   = 5053
MAX_VOLT    = 7.4

# [FIX-BRIDGE-1] Default host — resolve otomatis ke Windows host dari Docker
DEFAULT_SEND_HOST = 'host.docker.internal'

# [FIX-BRIDGE-3] Log hanya setiap N paket
LOG_EVERY_N_PACKETS = 100


class LabViewBridge(Node):

    def __init__(self):
        super().__init__('labview_bridge')

        self.declare_parameter('listen_port', LISTEN_PORT)
        self.declare_parameter('send_port',   SEND_PORT)
        # [FIX-BRIDGE-1] Parameter send_host
        self.declare_parameter('send_host',   DEFAULT_SEND_HOST)

        lport          = self.get_parameter('listen_port').value
        self.sport     = self.get_parameter('send_port').value
        self.send_host = self.get_parameter('send_host').value

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LISTEN_IP, lport))
        self.sock.settimeout(0.5)

        # [FIX-BRIDGE-1] Set alamat tujuan langsung dari parameter
        # Tidak lagi bergantung pada addr[0] dari recvfrom
        self._resolve_send_addr()

        # Buffer data robot
        self.data_buf = {
            'x': 0.0, 'y': 0.0, 'th': 0.0, 'v': 0.0, 'w': 0.0,
            'ex': 0.0, 'ey': 0.0, 'eth': 0.0,
            'volt_L': 0.0, 'volt_R': 0.0,
            'vr': 0.0, 'tau_L': 0.0, 'tau_R': 0.0,
            'sim_type': 0,
        }

        # [FIX-BRIDGE-3] Counter untuk rate-limit logging
        self._rx_count  = 0
        self._prev_mode = -1

        # Publishers
        self.pub_lv   = self.create_publisher(
            Float64MultiArray, '/labview_params', 10)
        self.pub_ctrl = self.create_publisher(
            Int32, '/sim_control', 10)

        # Subscribers
        self.create_subscription(
            Float64MultiArray, '/robot_state',
            self._cb_robot_state, 10)
        self.create_subscription(
            Float64MultiArray, '/error_state',
            self._cb_error_state, 10)
        self.create_subscription(
            Float64MultiArray, '/motor_voltage',
            self._cb_motor_voltage, 10)
        self.create_subscription(
            Float64MultiArray, '/motor_state',
            self._cb_motor_state, 10)

        # UDP listen thread
        self._stop_udp = False
        self.udp_thread = threading.Thread(
            target=self._udp_loop, daemon=True)
        self.udp_thread.start()

        # [FIX-BRIDGE-2] Timer kirim 100Hz (bukan 10Hz!)
        self.timer_send = self.create_timer(0.01, self._send_to_labview)

        self.get_logger().info(
            'labview_bridge siap [FIX-BRIDGE-1,2,3,4].\n'
            '  Listen UDP : %s:%d\n'
            '  Send host  : %s:%d  [FIX-BRIDGE-1]\n'
            '  Send rate  : 100Hz (bukan 10Hz)  [FIX-BRIDGE-2]\n'
            '  Log rate   : 1x per %d paket  [FIX-BRIDGE-3]\n'
            '  CSV format : 14 nilai (drift dihapus)  [FIX-BRIDGE-4]' %
            (LISTEN_IP, lport,
             self.send_host, self.sport,
             LOG_EVERY_N_PACKETS))

    def _resolve_send_addr(self):
        """[FIX-BRIDGE-1] Resolve send_host ke IP."""
        try:
            ip = socket.gethostbyname(self.send_host)
            self.labview_addr = (ip, self.sport)
            self.get_logger().info(
                '[FIX-BRIDGE-1] Send addr: %s → %s:%d' %
                (self.send_host, ip, self.sport))
        except socket.gaierror:
            # Fallback ke gateway Docker default
            fallback = '172.17.0.1'
            self.labview_addr = (fallback, self.sport)
            self.get_logger().warn(
                '[FIX-BRIDGE-1] Gagal resolve %s, fallback ke %s:%d' %
                (self.send_host, fallback, self.sport))

    # ── Subscriber callbacks ─────────────────────────────────────
    def _cb_robot_state(self, msg):
        if len(msg.data) >= 5:
            d = msg.data
            b = self.data_buf
            b['x'] = d[0]; b['y'] = d[1]; b['th'] = d[2]
            b['v'] = d[3]; b['w'] = d[4]

    def _cb_error_state(self, msg):
        if len(msg.data) >= 4:
            d = msg.data
            b = self.data_buf
            b['ex'] = d[0]; b['ey'] = d[1]; b['eth'] = d[2]
            b['vr'] = d[3]

    def _cb_motor_voltage(self, msg):
        if len(msg.data) >= 2:
            self.data_buf['volt_L'] = msg.data[0]
            self.data_buf['volt_R'] = msg.data[1]

    def _cb_motor_state(self, msg):
        if len(msg.data) >= 2:
            self.data_buf['tau_R'] = msg.data[0]
            self.data_buf['tau_L'] = msg.data[1]

    # ── UDP receive loop ─────────────────────────────────────────
    def _udp_loop(self):
        """Thread terima UDP dari LabVIEW."""
        while not self._stop_udp:
            try:
                data, addr = self.sock.recvfrom(1024)

                if len(data) < 64:
                    continue

                # Parse 8 double big-endian
                u = struct.unpack('>dddddddd', data[0:64])
                kp_R       = u[0]; ki_R = u[1]; kd_R = u[2]
                kp_L       = u[3]; ki_L = u[4]; kd_L = u[5]
                mode_reset = int(u[6])
                sim_type   = int(u[7])

                self._rx_count += 1

                # [FIX-BRIDGE-3] Log hanya setiap N paket atau saat mode berubah
                is_important = (
                    mode_reset in (9, 99) or
                    sim_type != self._prev_mode or
                    self._rx_count % LOG_EVERY_N_PACKETS == 1
                )
                if is_important:
                    mode_str = 'KBBC+PID' if sim_type == 1 else 'KBBC Only'
                    self.get_logger().info(
                        '[UDP RX #%d dari %s] mode=%s | '
                        'KpR=%.4f KiR=%.4f KdR=%.4f | reset=%d%s' % (
                            self._rx_count, addr[0], mode_str,
                            kp_R, ki_R, kd_R,
                            mode_reset,
                            '  ← RESET!' if mode_reset == 9 else
                            ('  ← STOP!' if mode_reset == 99 else '')
                        ))
                    self._prev_mode = sim_type

                # Publish /labview_params
                msg = Float64MultiArray()
                msg.data = [kp_R, ki_R, kd_R,
                            kp_L, ki_L, kd_L,
                            float(mode_reset), float(sim_type)]
                self.pub_lv.publish(msg)

                # Publish /sim_control untuk reset/stop
                if mode_reset in (9, 99):
                    ctrl = Int32()
                    ctrl.data = mode_reset
                    self.pub_ctrl.publish(ctrl)

                self.data_buf['sim_type'] = sim_type

            except socket.timeout:
                pass
            except Exception as e:
                self.get_logger().warn('UDP error: %s' % str(e))

    # ── Send ke LabVIEW ──────────────────────────────────────────
    def _send_to_labview(self):
        """[FIX-BRIDGE-2] Kirim data ke LabVIEW @ 100Hz."""
        if self.labview_addr is None:
            return

        b   = self.data_buf
        pwL = b['volt_L'] * (100.0 / MAX_VOLT)
        pwR = b['volt_R'] * (100.0 / MAX_VOLT)

        # [FIX-BRIDGE-4] 14 nilai (hapus drift, sesuai LabVIEW format)
        msg_str = (
            '%.3f,%.3f,%.3f,'    # x, y, theta
            '%.3f,%.3f,'         # v, w
            '%.3f,%.3f,%.3f,'    # ex, ey, eth
            '%.3f,%.3f,'         # pwmL, pwmR
            '%.3f,%.4f,%.4f,'    # vr, tau_L, tau_R
            '%d'                 # sim_type  ← 14 nilai total
        ) % (
            b['x'], b['y'], b['th'],
            b['v'], b['w'],
            b['ex'], b['ey'], b['eth'],
            pwL, pwR,
            b['vr'], b['tau_L'], b['tau_R'],
            b['sim_type']
        )

        try:
            self.sock.sendto(msg_str.encode(), self.labview_addr)
        except Exception as e:
            pass   # Tidak log agar tidak spam

    def destroy_node(self):
        self._stop_udp = True
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LabViewBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
