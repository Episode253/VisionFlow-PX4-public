import numpy as np

class KalmanFilter:
    def __init__(self, dt, INIT_POS_STD, INIT_VEL_STD, ACCEL_STD, GPS_POS_STD):

        self.dt = dt

        # 初始状态 [x, y, vx, vy]^T
        self.x = np.zeros((4, 1))

        # 状态估计协方差矩阵 (P)
        self.P = np.diag([
            INIT_POS_STD ** 2,
            INIT_POS_STD ** 2,
            INIT_VEL_STD ** 2,
            INIT_VEL_STD ** 2
        ])

        # 状态转移矩阵 (F)
        self.F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1]
        ])

        # 过程噪声映射矩阵 (L)
        self.L = np.array([
            [0.5 * dt ** 2, 0],
            [0,             0.5 * dt ** 2],
            [dt,            0],
            [0,             dt]
        ])

        # 加速度扰动协方差 (q)
        self.q = np.diag([ACCEL_STD ** 2, ACCEL_STD ** 2])

        # 最终的过程噪声协方差矩阵 (Q = L * q * L^T)
        self.Q = self.L @ self.q @ self.L.T

        # 测量映射矩阵 (H) - 我们只能观测到位置 x 和 y
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # 测量协方差矩阵 (R)
        self.R = np.diag([GPS_POS_STD ** 2, GPS_POS_STD ** 2])

        # 缓存单位矩阵以供 update 使用
        self.I = np.eye(4)

    def predict(self):
        # 1. 预测下一步状态: x = F * x
        self.x = self.F @ self.x

        # 2. 预测误差协方差: P = F * P * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

        # 提取标量结果并返回
        return float(self.x[0, 0]), float(self.x[1, 0])

    def update(self, z):
        # 防御性编程：强制将测量值 z 转换为 2x1 列向量，防止 Numpy 广播导致维度爆炸
        z = np.array(z).reshape(2, 1)

        # 1. 计算测量残差 (Innovation): y = z - H * x
        y = z - (self.H @ self.x)

        # 2. 计算残差协方差: S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R

        # 3. 计算卡尔曼增益: K = P * H^T * S^-1
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 4. 更新状态估计: x = x + K * y
        self.x = self.x + K @ y

        # 5. 更新误差协方差 (使用 Joseph form)
        I_KH = self.I - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

        # 5. 更新误差协方差: P = (I - K * H) * P
        # self.P = (self.I - K @ self.H) @ self.P

        # 提取标量结果并返回
        return float(self.x[0, 0]), float(self.x[1, 0])
