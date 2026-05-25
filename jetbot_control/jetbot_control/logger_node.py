"""
================================================================
logger_node.py  —  ROS 2 Node: CSV Logger + Debug Print
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
FUNGSI:
  Mengumpulkan data dari semua topic dan menyimpannya ke CSV.
  Juga mencetak debug info ke terminal setiap 1 detik.

SUBSCRIBE:
  /reference_state  [Float64MultiArray]  xr,yr,thr,vr,wr,time
  /robot_state      [Float64MultiArray]  x,y,th,v,w
  /error_state      [Float64MultiArray]  ex,ey,eth,vr,wr
  /motor_state      [Float64MultiArray]  tauR,tauL,iR,iL,omR,omL
  /motor_voltage    [Float64MultiArray]  volt_L,volt_R
  /odom_est         [Float64MultiArray]  xe,ye,the,ve,we,omRe,omLe,drift
  /labview_params   [Float64MultiArray]  kpR,kiR,kdR,kpL,kiL,kdL,mode,type
  /sim_control      [Int32]

FORMAT CSV (sama dengan simulasi sebelumnya):
  Time, X_true, Y_true, Theta_true,
  X_est, Y_est, Theta_est,
  V_rob, W_rob,
  Ex, Ey, Eth,
  Vr, Wr,
  Volt_L, Volt_R,
  Tau_L, Tau_R,
  iR, iL,
  OmR_plant, OmL_plant,
  OmR_enc, OmL_enc,
  Drift_m, SimType
================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
import csv
import time
import math


class LoggerNode(Node):

    def __init__(self):
        super().__init__('logger_node')

        self.declare_parameter('debug_interval', 1.0)
        self.declare_parameter('dt', 0.01)
        self.debug_interval = self.get_parameter('debug_interval').value
        self.dt             = self.get_parameter('dt').value

        # ── Buka file CSV ────────────────────────────────────
        fname = 'LOG_JETBOT_%d.csv' % int(time.time())
        self.fcsv = open(fname, 'w', newline='')
        self.cw   = csv.writer(self.fcsv)
        self.cw.writerow([
            'Time',
            'X_true', 'Y_true', 'Theta_true',
            'X_est',  'Y_est',  'Theta_est',
            'V_rob',  'W_rob',
            'Ex', 'Ey', 'Eth',
            'Vr', 'Wr',
            'Volt_L', 'Volt_R',
            'Tau_L',  'Tau_R',
            'iR',     'iL',
            'OmR_plant', 'OmL_plant',
            'OmR_enc',   'OmL_enc',
            'Drift_m', 'SimType'
        ])
        self.get_logger().info('Log file: %s' % fname)

        # ── State buffer (diisi oleh subscriber) ─────────────
        self.buf = {
            'time':  0.0,
            'x':0.0, 'y':0.0, 'th':0.0, 'v':0.0, 'w':0.0,
            'xr':0.0,'yr':0.0,'thr':0.0,'vr':0.0,'wr':0.0,
            'ex':0.0,'ey':0.0,'eth':0.0,
            'volt_L':0.0,'volt_R':0.0,
            'tau_R':0.0,'tau_L':0.0,'iR':0.0,'iL':0.0,
            'omR_plant':0.0,'omL_plant':0.0,
            'xe':0.0,'ye':0.0,'the':0.0,
            'omR_enc':0.0,'omL_enc':0.0,'drift':0.0,
            'sim_type':0,
            'kp_R':0.0,'ki_R':0.0,'kd_R':0.0,
            'kp_L':0.0,'ki_L':0.0,'kd_L':0.0,
        }
        self.last_debug = 0.0
        self.running    = True

        # ── Subscribers ──────────────────────────────────────
        self.create_subscription(
            Float64MultiArray, '/reference_state',
            lambda m: self._upd_ref(m), 10)
        self.create_subscription(
            Float64MultiArray, '/robot_state',
            lambda m: self._upd_state(m), 10)
        self.create_subscription(
            Float64MultiArray, '/error_state',
            lambda m: self._upd_error(m), 10)
        self.create_subscription(
            Float64MultiArray, '/motor_state',
            lambda m: self._upd_motor(m), 10)
        self.create_subscription(
            Float64MultiArray, '/motor_voltage',
            lambda m: self._upd_volt(m), 10)
        self.create_subscription(
            Float64MultiArray, '/odom_est',
            lambda m: self._upd_odom(m), 10)
        self.create_subscription(
            Float64MultiArray, '/labview_params',
            lambda m: self._upd_lv(m), 10)
        self.create_subscription(
            Int32, '/sim_control',
            lambda m: self._cb_ctrl(m), 10)

        # ── Timer 10ms — tulis CSV ───────────────────────────
        self.timer = self.create_timer(self.dt, self._timer_cb)

    # ── Update buffer dari setiap topic ──────────────────────
    def _upd_ref(self, msg):
        d = msg.data
        if len(d) >= 6:
            self.buf['xr']=d[0]; self.buf['yr']=d[1]; self.buf['thr']=d[2]
            self.buf['vr']=d[3]; self.buf['wr']=d[4]; self.buf['time']=d[5]

    def _upd_state(self, msg):
        d = msg.data
        if len(d) >= 5:
            self.buf['x']=d[0]; self.buf['y']=d[1]; self.buf['th']=d[2]
            self.buf['v']=d[3]; self.buf['w']=d[4]

    def _upd_error(self, msg):
        d = msg.data
        if len(d) >= 3:
            self.buf['ex']=d[0]; self.buf['ey']=d[1]; self.buf['eth']=d[2]

    def _upd_motor(self, msg):
        d = msg.data
        if len(d) >= 6:
            self.buf['tau_R']=d[0]; self.buf['tau_L']=d[1]
            self.buf['iR']=d[2];    self.buf['iL']=d[3]
            self.buf['omR_plant']=d[4]; self.buf['omL_plant']=d[5]

    def _upd_volt(self, msg):
        d = msg.data
        if len(d) >= 2:
            self.buf['volt_L']=d[0]; self.buf['volt_R']=d[1]

    def _upd_odom(self, msg):
        d = msg.data
        if len(d) >= 8:
            self.buf['xe']=d[0];    self.buf['ye']=d[1]
            self.buf['the']=d[2]
            self.buf['omR_enc']=d[5]; self.buf['omL_enc']=d[6]
            self.buf['drift']=d[7]

    def _upd_lv(self, msg):
        d = msg.data
        if len(d) >= 8:
            self.buf['kp_R']=d[0]; self.buf['ki_R']=d[1]; self.buf['kd_R']=d[2]
            self.buf['kp_L']=d[3]; self.buf['ki_L']=d[4]; self.buf['kd_L']=d[5]
            self.buf['sim_type']=int(d[7])

    def _cb_ctrl(self, msg):
        if msg.data == 99:
            self.running = False
            self.get_logger().info('STOP — menutup file CSV')
            self.fcsv.flush(); self.fcsv.close()

    # ── Timer: tulis satu baris CSV + debug ──────────────────
    def _timer_cb(self):
        if not self.running:
            return

        b = self.buf
        self.cw.writerow([
            '%.3f' % b['time'],
            '%.4f' % b['x'],    '%.4f' % b['y'],    '%.4f' % b['th'],
            '%.4f' % b['xe'],   '%.4f' % b['ye'],   '%.4f' % b['the'],
            '%.4f' % b['v'],    '%.4f' % b['w'],
            '%.4f' % b['ex'],   '%.4f' % b['ey'],   '%.4f' % b['eth'],
            '%.4f' % b['vr'],   '%.4f' % b['wr'],
            '%.3f' % b['volt_L'], '%.3f' % b['volt_R'],
            '%.5f' % b['tau_L'],  '%.5f' % b['tau_R'],
            '%.4f' % b['iR'],     '%.4f' % b['iL'],
            '%.3f' % b['omR_plant'], '%.3f' % b['omL_plant'],
            '%.3f' % b['omR_enc'],   '%.3f' % b['omL_enc'],
            '%.4f' % b['drift'],
            '%d'   % b['sim_type']
        ])

        # ── Debug print setiap 1 detik ───────────────────────
        if b['time'] - self.last_debug >= self.debug_interval:
            self.last_debug = b['time']
            mode_str = 'KBBC+PID' if b['sim_type'] == 1 else 'KBBC Only'
            self.get_logger().info(
                '\n%s\n'
                '  t=%7.2fs  |  Mode: %s\n'
                '  Gains R : Kp=%.4f  Ki=%.4f  Kd=%.4f\n'
                '  Gains L : Kp=%.4f  Ki=%.4f  Kd=%.4f\n'
                '  Error   : ex=%+.4f m  ey=%+.4f m  eth=%+.2f deg\n'
                '  v_rob=%.4f m/s   w_rob=%.5f rad/s\n'
                '  Volt_R=%.3fV  Volt_L=%.3fV\n'
                '  Torque  : tauR=%+.3f mNm  tauL=%+.3f mNm\n'
                '  Current : iR=%+.1f mA  iL=%+.1f mA\n'
                '  Drift   : %.4f m\n'
                '%s' % (
                    '-'*60, b['time'], mode_str,
                    b['kp_R'], b['ki_R'], b['kd_R'],
                    b['kp_L'], b['ki_L'], b['kd_L'],
                    b['ex'], b['ey'], math.degrees(b['eth']),
                    b['v'], b['w'],
                    b['volt_R'], b['volt_L'],
                    b['tau_R']*1000, b['tau_L']*1000,
                    b['iR']*1000, b['iL']*1000,
                    b['drift'],
                    '-'*60
                ))


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.running:
            node.fcsv.flush()
            node.fcsv.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
