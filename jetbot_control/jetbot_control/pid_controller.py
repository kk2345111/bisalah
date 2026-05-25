"""
================================================================
pid_controller.py  —  PID Inner Loop Controller (Motor Speed)
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
Referensi: Demirbaş & Kalyoncu (2017)
  Struktur: Feed-Forward + PID Correction (sesuai jurnal)

[FIX-PID-1] PERBAIKAN KRITIS — Feed-Forward ditambahkan kembali:
  MASALAH SEBELUMNYA (code lama):
    volt = Kp·eR + Ki·∫eR + Kd·ėR   (pure PID, no feed-forward!)
    → saat err→0 (steady state), volt → 0 → motor melambat → osilasi

  SEKARANG (diperbaiki):
    volt = Kb·N·ω_req  +  Kp·eR + Ki·∫eR + Kd·ėR
           ↑ feed-forward     ↑ PID correction
    → Feed-forward menjaga kecepatan, PID hanya koreksi error kecil

[FIX-PID-2] Offset arus (I_BIAS):
  MASALAH SEBELUMNYA (KBBC Only):
    volt_ff = Kb·N·ω_req  →  i_ss = (volt_ff - Kb·N·ω_act)/Ra
    Saat steady state (ω_act = ω_req): i_ss = 0 → torsi = 0!
  SEKARANG:
    volt_ff = Kb·N·ω_req + Ra·I_BIAS  →  i_ss = I_BIAS saat steady

[FIX-PID-3] Domain error yang benar:
  MASALAH SEBELUMNYA:
    pre = Kb·N = 0.72  (mengubah rad/s → V sebelum masuk PID)
    → Kp yang diinput LabVIEW berdimensi ambigu
  SEKARANG:
    Error tetap dalam rad/s, Kp dalam V/(rad/s)
    volt_pid = Kp·(ω_req - ω_act) + Ki·∫... + Kd·...
    Kp yang wajar: 0.05–0.5 V/(rad/s)

Gunakan sim_type=0 untuk KBBC Only (dengan I_BIAS, tanpa PID).
Gunakan sim_type=1 untuk KBBC+PID (feed-forward + PID).
================================================================
"""

# ── Parameter motor (harus sama dengan robot_plant.py) ──────────
K_b  = 0.015    # Back-EMF constant (V·s/rad)
N_g  = 48.0     # Gear ratio
R_a  = 3.0      # Resistansi armature (Ohm)

MAX_VOLT     = 7.4   # Batas voltase (V)
MAX_CURRENT  = 1.2   # Batas arus TB6612FNG (A)

# [FIX-PID-2] Offset arus steady-state (memastikan torsi ≠ 0)
I_BIAS = 0.15   # A  (menghasilkan τ = N·Kt·I_BIAS ≈ 0.108 Nm)

# ── Default gains ─────────────────────────────────────────────
# Kp dalam V/(rad/s) — BUKAN domain volt seperti code lama!
KP_DEFAULT = 0.1    # V/(rad/s)
KI_DEFAULT = 0.02   # V/(rad/s·s)
KD_DEFAULT = 0.005  # V·s/(rad/s)

# Filter derivative (low-pass time constant)
RC_DEFAULT = 0.05   # s


class MotorPIDController:
    """
    PID Controller untuk satu motor DC.

    Mode:
      sim_type=0 → KBBC Only: output = feed-forward + I_BIAS offset
      sim_type=1 → KBBC+PID:  output = feed-forward + PID correction
    """

    def __init__(self, Kp=KP_DEFAULT, Ki=KI_DEFAULT, Kd=KD_DEFAULT,
                 RC=RC_DEFAULT, side="R"):
        self.Kp   = Kp
        self.Ki   = Ki
        self.Kd   = Kd
        self.RC   = RC
        self.side = side
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._last_err = 0.0
        self._last_d   = 0.0

    def update_gains(self, Kp: float, Ki: float, Kd: float):
        """Update gains dari LabVIEW (real-time)."""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def compute(self, w_req: float, w_act: float,
                sim_type: int, dt: float) -> float:
        """
        Hitung voltase output untuk motor.

        Args:
            w_req    : kecepatan roda yang diminta (rad/s)
            w_act    : kecepatan roda aktual dari plant (rad/s)
            sim_type : 0=KBBC Only, 1=KBBC+PID
            dt       : control step (s)

        Returns:
            volt : tegangan motor (V), sudah di-clamp ke ±MAX_VOLT
        """
        # [FIX-PID-2] Feed-forward + I_BIAS (selalu ada di kedua mode)
        volt_ff = K_b * N_g * w_req + R_a * I_BIAS * (1.0 if w_req >= 0 else -1.0)

        if sim_type == 0:
            # ── KBBC Only: hanya feed-forward ──────────────────
            volt = volt_ff

        else:
            # ── KBBC+PID: feed-forward + PID correction ─────────
            # [FIX-PID-3] Error dalam rad/s (domain asli)
            err = w_req - w_act

            # Integral (dengan anti-windup clamp)
            self._integral += err * dt
            i_max = MAX_VOLT / max(self.Ki, 1e-9)
            self._integral = max(-i_max, min(i_max, self._integral))

            # Derivative dengan low-pass filter
            if dt > 0:
                alpha   = dt / (self.RC + dt)
                d_raw   = (err - self._last_err) / dt
                d_filt  = alpha*d_raw + (1.0-alpha)*self._last_d
            else:
                d_filt = self._last_d

            self._last_err = err
            self._last_d   = d_filt

            # PID correction (dalam domain rad/s, Kp dalam V/(rad/s))
            volt_pid = self.Kp*err + self.Ki*self._integral + self.Kd*d_filt

            # [FIX-PID-1] Feed-forward + PID (sesuai jurnal Simulink)
            volt = volt_ff + volt_pid

        return max(-MAX_VOLT, min(MAX_VOLT, volt))

    @property
    def gains(self):
        return self.Kp, self.Ki, self.Kd


def scale_volt(vR_raw: float, vL_raw: float, max_v: float = MAX_VOLT) -> tuple:
    """
    [FIX-M1] Scaling proporsional saat saturasi.
    Pertahankan rasio volt_R/volt_L agar robot tetap bisa belok.
    """
    v_abs = max(abs(vR_raw), abs(vL_raw))
    if v_abs > max_v:
        s = max_v / v_abs
        return vR_raw*s, vL_raw*s
    return vR_raw, vL_raw


if __name__ == "__main__":
    pid = MotorPIDController(Kp=0.1, Ki=0.02, Kd=0.005)

    print("=== Test KBBC Only (sim_type=0) ===")
    for w in [0, 3, 7.75, 10]:
        v = pid.compute(w, w, sim_type=0, dt=0.01)
        i_ss = (v - K_b*N_g*w) / R_a
        print(f"  w_req={w:5.2f}: volt={v:.3f}V  i_ss={i_ss:.4f}A (I_BIAS termasuk)")

    print()
    print("=== Test KBBC+PID (sim_type=1) ===")
    pid.reset()
    pid.update_gains(0.1, 0.02, 0.005)
    w_req = 5.0; w_act = 0.0
    print(f"  {'Step':>4} | {'w_act':>7} | {'volt':>7} | {'err':>7}")
    for step in range(5):
        v = pid.compute(w_req, w_act, sim_type=1, dt=0.01)
        i_ss = (v - K_b*N_g*w_act) / R_a
        # Simplified dynamics
        tau = N_g*0.015*max(-1.2, min(1.2, i_ss))
        w_act = min(10.0, w_act + (tau/0.000015)*0.01)
        print(f"  {step+1:>4} | {w_act:>7.3f} | {v:>7.3f} | {w_req-w_act:>7.4f}")
