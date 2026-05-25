"""
================================================================
trajectory_generator.py  —  Diamond Trajectory Generator
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
Referensi: Demirbaş & Kalyoncu (2017), Persamaan 40

Sifat matematis yang PENTING:
  - vr = 0 di setiap sudut (t = π/2w, π/w, 3π/2w, 2π/w)
  - wr = 0 SEPANJANG WAKTU (curvature nol, tiap segmen lurus)
  - θ_r berubah 90° mendadak di tiap sudut (1 timestep)
  - Ini adalah KETERBATASAN KBBC: w_cmd ≈ 0 saat vr → 0
    karena w_cmd = wr + vr*(KY*ey + Kθ*sin(eθ))
================================================================
"""

import math

SCALE_K = 1.5    # meter
W_TRAJ  = 0.1   # rad/s  →  T = 62.83 s


class TrajectoryGenerator:
    def __init__(self, scale=SCALE_K, omega=W_TRAJ):
        self.k = scale
        self.w = omega

    def get_state(self, t: float) -> tuple:
        """
        Returns (xr, yr, thr, vr, wr) pada waktu t.
        """
        wt = self.w * t
        c  = math.cos(wt);  s  = math.sin(wt)
        sc = 1.0 if c >= 0 else -1.0
        ss = 1.0 if s >= 0 else -1.0

        xr   =  self.k * sc * c**2
        yr   =  self.k * ss * s**2
        dxr  = -self.k * self.w * sc * math.sin(2*wt)
        dyr  =  self.k * self.w * ss * math.sin(2*wt)
        ddxr = -2.0 * self.k * self.w**2 * sc * math.cos(2*wt)
        ddyr =  2.0 * self.k * self.w**2 * ss * math.cos(2*wt)

        vr  = math.sqrt(dxr**2 + dyr**2)
        thr = math.atan2(dyr, dxr)
        wr  = (dxr*ddyr - dyr*ddxr) / (vr**2 + 1e-9)  # selalu ≈ 0

        return xr, yr, thr, vr, wr

    @property
    def period(self):
        return 2.0 * math.pi / self.w

    def corner_times(self):
        T = self.period
        return [T/4, T/2, 3*T/4, T]


if __name__ == "__main__":
    traj = TrajectoryGenerator()
    print(f"Period={traj.period:.2f}s  Corners={[round(t,2) for t in traj.corner_times()]}")
    for t in [0, 7, 15.71, 20, 31.42, 47.13, 62.83]:
        xr,yr,thr,vr,wr = traj.get_state(t)
        print(f"  t={t:6.2f}: xr={xr:7.4f} yr={yr:7.4f} "
              f"thr={math.degrees(thr):8.2f}° vr={vr:.5f} wr={wr:.8f}")
