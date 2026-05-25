"""
================================================================
sensor_model.py  —  Model Sensor MH-Sensor-Series [v4]
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
Hardware:
  Sensor   : MH-Sensor-Series (IR Optocoupler)
  Piringan : 20 lubang per putaran roda
  Motor    : TT Motor 1:48  (N=48)
  Driver   : TB6612FNG

PENTING — Peran sensor ini di simulasi:
  EncoderModel  → estimasi kecepatan roda (untuk odometri)
  OdometryModel → estimasi posisi (X_est, Y_est) dari dead reckoning

  ⚠ Encoder TIDAK dipakai untuk feedback PID/KBBC!
    Feedback kecepatan roda ke PID diambil dari plant langsung
    (sesuai struktur Simulink jurnal Demirbaş 2017).
    Encoder hanya untuk logging kolom X_est, Y_est, Drift_m.
================================================================
"""

import math
from collections import deque

# ── Parameter (harus konsisten dengan robot_plant.py) ──────────
PPR    = 20       # Pulsa per putaran roda
R      = 0.03     # Radius roda (m)
L      = 0.065    # Setengah wheelbase (m)
V_THR  = 0.20     # Threshold voltase arah (V)
K_b    = 0.015    # Back-EMF constant
N_g    = 48.0     # Gear ratio
MAX_V  = 7.4      # Max voltase (V)
OM_MAX = MAX_V / (K_b * N_g)   # ≈ 10.28 rad/s

GRACE  = 3        # Grace period sebelum set omega=0


class EncoderModel:
    """
    Model sensor MH-Sensor-Series 20 lubang.

    Menggunakan sliding window 300ms untuk estimasi kecepatan.
    Arah ditentukan dari polaritas voltase (≥ V_THR).
    """

    WINDOW = 30   # 30 × 10ms = 300ms

    def __init__(self, ppr: int = PPR, side: str = "R"):
        self.ppr  = ppr
        self.side = side
        self._init_state()

    def _init_state(self):
        self.accum          = 0.0
        self.buf            = deque(maxlen=self.WINDOW)
        self.omega_measured = 0.0
        self.last_dir       = 1.0
        self.total_pulses   = 0
        self.missed         = 0
        self.calls          = 0
        self._grace         = 0
        self._last_valid    = 0.0

    def reset(self):
        self._init_state()

    def update(self, omega_true: float, volt_cmd: float, dt: float) -> float:
        """
        Update satu control step (10ms).

        Args:
            omega_true : kecepatan roda dari plant (rad/s)
            volt_cmd   : tegangan motor (V) untuk arah
            dt         : step waktu (s)

        Returns:
            omega_measured : estimasi kecepatan encoder (rad/s)
        """
        self.calls += 1

        # Tentukan arah dari voltase
        if   volt_cmd >  V_THR: self.last_dir =  1.0
        elif volt_cmd < -V_THR: self.last_dir = -1.0

        # Akumulasi pulsa
        self.accum += abs(omega_true) * (self.ppr / (2.0 * math.pi)) * dt
        n = int(self.accum)
        self.accum        -= n
        self.total_pulses += n
        if n == 0: self.missed += 1

        # Sliding window (signed)
        self.buf.append(n * self.last_dir)
        total = sum(self.buf)
        t_win = len(self.buf) * dt

        if abs(total) > 0:
            raw = total * (2.0 * math.pi / self.ppr) / t_win
            raw = max(-OM_MAX, min(OM_MAX, raw))
            self.omega_measured = raw
            self._last_valid    = raw
            self._grace         = 0
        else:
            if self._grace < GRACE:
                self.omega_measured = self._last_valid
                self._grace        += 1
            else:
                self.omega_measured = 0.0

        return self.omega_measured

    @property
    def dead_zone_pct(self) -> float:
        return 100.0 * self.missed / max(1, self.calls)

    def info(self) -> dict:
        dth = 2.0 * math.pi / self.ppr
        return {
            "ppr"           : self.ppr,
            "delta_deg"     : round(math.degrees(dth), 1),
            "delta_dist_cm" : round(dth * R * 100, 3),
            "window_ms"     : self.WINDOW * 10,
            "OM_MAX_rads"   : round(OM_MAX, 3),
            "dead_zone_pct" : round(self.dead_zone_pct, 1)
        }


class OdometryModel:
    """
    Estimasi posisi via Dead Reckoning dari encoder.

    Persamaan differential drive:
      v_enc = R/2    × (ωR + ωL)
      w_enc = R/(2L) × (ωR − ωL)
      x    += v_enc × cos(θ) × dt
      y    += v_enc × sin(θ) × dt
      θ    += w_enc × dt
    """

    def __init__(self, x0=0.0, y0=0.0, th0=0.0):
        self.x            = x0
        self.y            = y0
        self.th           = th0
        self.v            = 0.0
        self.w            = 0.0
        self.drift_accum  = 0.0

    def reset(self, x0=0.0, y0=0.0, th0=0.0):
        self.x  = x0; self.y  = y0; self.th = th0
        self.v  = 0.0; self.w = 0.0
        self.drift_accum = 0.0

    def update(self, omR: float, omL: float, dt: float) -> tuple:
        """Returns: (x_est, y_est, th_est, v_est, w_est)"""
        self.v   = (R / 2.0)      * (omR + omL)
        self.w   = (R / (2.0*L))  * (omR - omL)
        self.x  += self.v * math.cos(self.th) * dt
        self.y  += self.v * math.sin(self.th) * dt
        self.th += self.w * dt
        self.th  = (self.th + math.pi) % (2.0*math.pi) - math.pi
        return self.x, self.y, self.th, self.v, self.w

    def drift(self, x_true: float, y_true: float, th_true: float) -> float:
        """Euclidean drift vs ground truth."""
        ep = math.sqrt((self.x - x_true)**2 + (self.y - y_true)**2)
        self.drift_accum += ep
        return ep


if __name__ == "__main__":
    enc  = EncoderModel(ppr=20, side="R")
    odom = OdometryModel()
    dt   = 0.01
    print(f"Encoder info: {enc.info()}")
    for i in range(40):
        wt = 0.2 / R
        wm = enc.update(wt, 3.0, dt)
        x, y, th, v, w = odom.update(wm, wm, dt)
    print(f"After 0.4s at v=0.2m/s: x={x:.4f}m  dead_zone={enc.dead_zone_pct:.1f}%")
