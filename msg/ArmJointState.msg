uint64 timestamp            # time since system start (microseconds)

float32[6] q                # 关节角 [rad], 顺序 A~F
float32[6] qd               # 关节速度 [rad/s] (可选, 填0表示未知)
