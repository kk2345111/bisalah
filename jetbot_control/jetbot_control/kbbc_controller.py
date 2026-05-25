"""
================================================================
kbbc_controller.py  —  Kinematic Based Backstepping Controller
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
Referensi: Kanayama et al. (1990), dikutip di Demirbaş (2017)
  Eq.30 — Error postur (dalam frame robot)
  Eq.31 — Hukum kontrol KBBC
  Eq.38 — Fungsi Lyapunov (bukti stabilitas)

Hukum kontrol (Eq.31):
  v  = vr·cos(eθ) + Kx·ex
  ω  = ωr + vr·(Ky·ey + Kθ·sin(eθ))

[FIX-KBBC-1] Dead zone saat vr = 0:
  MASALAH: saat vr→0 di sudut, w_cmd = wr + 0·(...) = 0
           Robot tidak bisa koreksi orientasi!
  SOLUSI:  Tambah term koreksi orientasi independen dari vr:
           w_cmd += K_DIRECT · eθ
           Aktif di semua kondisi, termasuk saat vr=0.

Stabilitas:
  Lyapunov: V = (ex²+ey²)/2 + (1-cos(eθ))/Ky
  V̇ ≤ 0 terbukti jika Kx, Ky, Kθ > 0 dan vr, ωr kontinu.
================================================================
"""

import math


# ── Default gains (Demirbaş 2017) ──────────────────────────────
KX_DEFAULT      = 0.5
KY_DEFAULT      = 3.0
KTHETA_DEFAULT  = 0.5
KDIRECT_DEFAULT = 0.3   # [FIX-KBBC-1] Gain koreksi orientasi independen

# ── Batas kecepatan ─────────────────────────────────────────────
V_MAX = 0.26    # m/s   (batas hardware JetBot)
W_MAX = 1.5     # rad/s

# ── Rate limiter ────────────────────────────────────────────────
A_LIN_MAX = 0.8   # m/s²
A_ANG_MAX = 1.5   # rad/s²  ← dinaikkan dari 1.0 agar corner lebih responsif


class KBBCController:
    """
    Kinematic Based Backstepping Controller (Kanayama 1990).

    Tambahan [FIX-KBBC-1]: K_direct untuk koreksi orientasi
    saat vr=0 (dead zone di sudut trajektori).
    """

    def __init__(self,
                 Kx=KX_DEFAULT, Ky=KY_DEFAULT,
                 Ktheta=KTHETA_DEFAULT, K_direct=KDIRECT_DEFAULT):
        self.Kx       = Kx
        self.Ky       = Ky
        self.Ktheta   = Ktheta
        self.K_direct = K_direct   # [FIX-KBBC-1]

        # Rate limiter state
        self._lv = 0.0
        self._lw = 0.0

    def reset(self):
        self._lv = 0.0
        self._lw = 0.0

    def compute_error(self, x: float, y: float, th: float,
                      xr: float, yr: float, thr: float) -> tuple:
        """
        Hitung error postur dalam frame robot (Eq.30).

        e_p = R(θ)^T × [xr-x, yr-y, θr-θ]
          ex  =  cos(θ)·(xr-x) + sin(θ)·(yr-y)
          ey  = -sin(θ)·(xr-x) + cos(θ)·(yr-y)
          eθ  = atan2(sin(θr-θ), cos(θr-θ))  ← normalisasi [-π,π]

        Returns:
            (ex, ey, eth)
        """
        c   = math.cos(th); s = math.sin(th)
        dxg = xr - x;       dyg = yr - y
        ex  =  c*dxg + s*dyg
        ey  = -s*dxg + c*dyg
        eth = math.atan2(math.sin(thr-th), math.cos(thr-th))  # [FIX-M4]
        return ex, ey, eth

    def compute_velocity(self, ex: float, ey: float, eth: float,
                         vr: float, wr: float,
                         dt: float) -> tuple:
        """
        Hitung perintah kecepatan (v_cmd, w_cmd) dari Eq.31 + rate limiter.

        [FIX-KBBC-1] w_cmd mendapat tambahan K_direct·eθ
        agar robot bisa koreksi orientasi bahkan saat vr=0.

        Returns:
            (v_cmd, w_cmd)  — sudah di-clamp dan rate-limited
        """
        # Eq.31 — Kanayama 1990
        v_raw = vr * math.cos(eth) + self.Kx * ex
        w_raw = wr + vr*(self.Ky*ey + self.Ktheta*math.sin(eth))

        # [FIX-KBBC-1] Term koreksi orientasi independen dari vr
        w_raw += self.K_direct * eth

        # Rate limiter [FIX-M5]
        if dt > 0:
            al = max(-A_LIN_MAX, min(A_LIN_MAX, (v_raw - self._lv)/dt))
            aa = max(-A_ANG_MAX, min(A_ANG_MAX, (w_raw - self._lw)/dt))
            v_cmd = self._lv + al * dt
            w_cmd = self._lw + aa * dt
        else:
            v_cmd = v_raw
            w_cmd = w_raw

        self._lv = v_cmd
        self._lw = w_cmd

        # Clamp ke batas hardware
        v_cmd = max(-V_MAX, min(V_MAX, v_cmd))
        w_cmd = max(-W_MAX, min(W_MAX, w_cmd))

        return v_cmd, w_cmd

    def inverse_kinematics(self, v_cmd: float, w_cmd: float,
                           R: float, L: float) -> tuple:
        """
        Konversi (v, ω) → (ω_R, ω_L) kecepatan roda (rad/s).
          ω_R = (v + ω·L) / R
          ω_L = (v - ω·L) / R
        """
        w_R = (v_cmd + w_cmd * L) / R
        w_L = (v_cmd - w_cmd * L) / R
        return w_R, w_L


if __name__ == "__main__":
    kbbc = KBBCController()
    # Test: robot di (0,0,0), referensi di (0.5, 0, 0)
    ex,ey,eth = kbbc.compute_error(0,0,0, 0.5,0,0)
    v,w = kbbc.compute_velocity(ex,ey,eth, 0.2, 0.0, 0.01)
    print(f"Test1: ex={ex:.3f} ey={ey:.3f} eth={math.degrees(eth):.1f}° "
          f"v_cmd={v:.4f} w_cmd={w:.4f}")

    # Test dead zone fix: vr=0 tapi eth besar
    ex,ey,eth = kbbc.compute_error(0,0,0, 0,0,math.pi/2)
    kbbc.reset()
    v,w = kbbc.compute_velocity(ex,ey,eth, 0.0, 0.0, 0.01)
    print(f"Test2 (vr=0, eth=90°): v_cmd={v:.4f} w_cmd={w:.4f} "
          f"← [FIX-KBBC-1] w_cmd≠0")
