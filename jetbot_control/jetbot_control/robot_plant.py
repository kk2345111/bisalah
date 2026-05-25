"""
================================================================
robot_plant.py  —  Lagrangian Robot Plant + DC Motor Model  [FIXED]
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
Referensi: Demirbaş & Kalyoncu (2017)

PERBAIKAN [FIX-PLANT-2] — SIM_DT dinaikkan 0.0001 → 0.001:
  MASALAH LAMA: SIM_DT = 0.0001s (0.1ms) menghasilkan 100 sub-step
  RK4 per control step (10ms). Dengan 100Hz simulasi:
    100 sub-steps × 100Hz = 10.000 np.linalg.solve() per detik!
  Di ARM64 Docker (QEMU emulation), ini menyebabkan CPU 1984%.

  ANALISIS AKURASI: tau_e motor (konstanta waktu elektrik) =
    La/Ra ≈ 0.0001H/3.0Ω = 0.033ms
  Tapi di model ini kita pakai steady-state arus (tanpa La),
  jadi dinamika cepat elektrik tidak ada. Tau mekanik:
    J/b ≈ 0.00001/0.001 = 10ms >> SIM_DT baru 1ms
  → SIM_DT = 1ms CUKUP AKURAT untuk dinamika mekanik robot.

  HASIL: 10 sub-steps × 100Hz = 1.000 linalg.solve() per detik
  (10× lebih sedikit). CPU turun drastis, stabilitas RK4 tetap terjaga
  karena SIM_DT=1ms << tau mekanik=10ms.

[FIX-PLANT-1] (sudah ada): Model arus steady-state tanpa La.
================================================================
"""

import math
import numpy as np

# Hardware JetBot
R    = 0.03
L    = 0.065
d    = 0.02

m_d  = 1.0
m_w  = 0.035
m    = m_d + 2.0 * m_w

I_d  = 0.003
I_m  = 0.00001
I_w  = 0.000015
I    = I_d + m_d * d**2 + 2.0 * m_w * L**2 + 2.0 * I_m

# Parameter motor DC
R_a  = 3.0
K_b  = 0.015
K_t  = 0.015
N_g  = 48.0

# Batas hardware
MAX_VOLT    = 7.4
MAX_CURRENT = 1.2
W_WHEEL_MAX = MAX_VOLT / (K_b * N_g)

# [FIX-PLANT-2] SIM_DT dinaikkan 10× untuk performa CPU
# Sebelumnya: 0.0001 (0.1ms) → 10.000 linalg.solve/detik
# Sekarang  : 0.001  (1ms)   →  1.000 linalg.solve/detik
SIM_DT = 0.001   # 1ms physics sub-step — masih akurat untuk dinamika robot


class RobotPlant:
    """
    Model dinamika Lagrangian robot JetBot.

    State: [x, y, θ, φ̇_R, φ̇_L]
    Input: volt_L, volt_R (tegangan motor, V)
    """

    def __init__(self, x0=0.0, y0=0.0, th0=0.0):
        self.q   = np.array([x0, y0, th0, 0.0, 0.0])
        self.dq  = np.zeros(5)
        self.i_R = 0.0
        self.i_L = 0.0
        self.v   = 0.0
        self.w_b = 0.0

    def reset(self, x0=0.0, y0=0.0, th0=0.0):
        self.__init__(x0, y0, th0)

    def _current_ss(self, volt: float, phi_dot: float) -> float:
        i = (volt - K_b * N_g * phi_dot) / R_a
        return max(-MAX_CURRENT, min(MAX_CURRENT, i))

    def _S(self, th: float) -> np.ndarray:
        c = math.cos(th); s = math.sin(th)
        S = np.zeros((5, 2))
        S[0, 0] = (R / (2*L)) * (L*c - d*s);  S[0, 1] = (R / (2*L)) * (L*c + d*s)
        S[1, 0] = (R / (2*L)) * (L*s + d*c);  S[1, 1] = (R / (2*L)) * (L*s - d*c)
        S[2, 0] = R / (2*L);                    S[2, 1] = -R / (2*L)
        S[3, 0] = 1.0;                           S[3, 1] =  0.0
        S[4, 0] = 0.0;                           S[4, 1] =  1.0
        return S

    def _dS(self, th: float, thd: float) -> np.ndarray:
        c = math.cos(th); s = math.sin(th)
        dS = np.zeros((5, 2))
        dS[0, 0] = (R / (2*L)) * (-L*s - d*c) * thd
        dS[1, 0] = (R / (2*L)) * ( L*c - d*s) * thd
        dS[0, 1] = (R / (2*L)) * (-L*s + d*c) * thd
        dS[1, 1] = (R / (2*L)) * ( L*c + d*s) * thd
        return dS

    def _deriv(self, st: np.ndarray, vL: float, vR: float) -> np.ndarray:
        th  = st[2]; phR = st[3]; phL = st[4]
        thd = (R / (2*L)) * (phR - phL)

        M = np.zeros((5, 5))
        M[0, 0] = m;  M[1, 1] = m
        M[0, 2] = -m_d * d * math.sin(th);  M[2, 0] = M[0, 2]
        M[1, 2] =  m_d * d * math.cos(th);  M[2, 1] = M[1, 2]
        M[2, 2] = I
        M[3, 3] = I_w;  M[4, 4] = I_w

        C = np.zeros((5, 5))
        C[0, 2] = -m_d * d * math.cos(th) * thd
        C[1, 2] = -m_d * d * math.sin(th) * thd

        B = np.zeros((5, 2));  B[3, 0] = 1.0;  B[4, 1] = 1.0

        S  = self._S(th)
        dS = self._dS(th, thd)

        iR  = self._current_ss(vR, phR)
        iL  = self._current_ss(vL, phL)
        tau = np.array([N_g * K_t * iR, N_g * K_t * iL])

        eta   = np.array([phR, phL])
        M_bar = S.T @ M  @ S
        V_bar = S.T @ (M @ dS + C @ S)
        B_bar = S.T @ B
        eta_dot = np.linalg.solve(M_bar, B_bar @ tau - V_bar @ eta)

        q_dot    = S @ eta
        dst      = np.zeros(5)
        dst[0:3] = q_dot[0:3]
        dst[3:5] = eta_dot
        return dst

    def step(self, vL: float, vR: float, control_dt: float) -> tuple:
        """
        Maju simulasi satu control step (control_dt detik).
        Menggunakan sub-step RK4 dengan SIM_DT=1ms [FIX-PLANT-2].

        Returns:
            (v, w, x, y, theta, tau_R, tau_L)
        """
        n_sub  = max(1, int(round(control_dt / SIM_DT)))
        dt_sub = control_dt / n_sub

        for _ in range(n_sub):
            st = np.array([self.q[0], self.q[1], self.q[2],
                           self.dq[3], self.dq[4]])

            k1 = self._deriv(st,                   vL, vR)
            k2 = self._deriv(st + 0.5*dt_sub*k1,   vL, vR)
            k3 = self._deriv(st + 0.5*dt_sub*k2,   vL, vR)
            k4 = self._deriv(st +     dt_sub*k3,   vL, vR)
            nst = st + (dt_sub / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

            self.q[0:3] = nst[0:3]
            self.q[2]   = (self.q[2] + math.pi) % (2*math.pi) - math.pi
            phR = nst[3]; phL = nst[4]

            self.i_R = self._current_ss(vR, phR)
            self.i_L = self._current_ss(vL, phL)

            S        = self._S(self.q[2])
            eta      = np.array([phR, phL])
            qk       = S @ eta
            self.dq[0:3] = qk[0:3]
            self.dq[3]   = phR
            self.dq[4]   = phL

        self.v   = (R / 2.0)     * (phR + phL)
        self.w_b = (R / (2.0*L)) * (phR - phL)
        tau_R    = N_g * K_t * self.i_R
        tau_L    = N_g * K_t * self.i_L

        return self.v, self.w_b, self.q[0], self.q[1], self.q[2], tau_R, tau_L

    @property
    def pos(self):
        return self.q[0], self.q[1], self.q[2]

    @property
    def wheel_speeds(self):
        return self.dq[3], self.dq[4]


if __name__ == '__main__':
    plant = RobotPlant()
    print('RobotPlant test [FIX-PLANT-2]: SIM_DT=0.001 (1ms)')
    import time
    t0 = time.time()
    for i in range(100):
        v, w, x, y, th, tR, tL = plant.step(3.0, 3.0, 0.01)
    dt = time.time() - t0
    print(f'  1s simulasi selesai dalam {dt*1000:.1f}ms wall-clock')
    print(f'  x={x:.4f}m  v={v:.4f}m/s  iR={plant.i_R:.4f}A  tR={tR:.5f}Nm')
