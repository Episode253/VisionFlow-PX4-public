#!/usr/bin/env python3
import subprocess
import time
import sys


# =========================
# 机械臂 6 个关节测试配置
# =========================
JOINTS = [
    {"name": "A", "topic": "/joint/1/position_cmd", "test_angle": 0.4},
    {"name": "B", "topic": "/joint/2/position_cmd", "test_angle": 0.4},
    {"name": "C", "topic": "/joint/3/position_cmd", "test_angle": 0.4},
    {"name": "D", "topic": "/joint/4/position_cmd", "test_angle": 0.4},
    {"name": "E", "topic": "/joint/5/position_cmd", "test_angle": 0.4},
    {"name": "F", "topic": "/joint/6/position_cmd", "test_angle": 0.4},
]

# 每个指令连续发布次数，避免单次消息偶发丢失
PUBLISH_REPEAT = 5

# 每次发布之间的间隔
PUBLISH_INTERVAL = 0.05

# 回零后等待时间
ZERO_WAIT = 1.0


def publish_joint_position(topic: str, position: float):
    """
    使用 gz topic 向指定关节发送位置控制指令。
    position 单位：rad
    """
    cmd = [
        "gz", "topic",
        "-t", topic,
        "-m", "gz.msgs.Double",
        "-p", f"data: {position}"
    ]

    for _ in range(PUBLISH_REPEAT):
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        if result.returncode != 0:
            print(f"\n[错误] 发布失败：{topic}")
            print(result.stderr)
            sys.exit(1)

        time.sleep(PUBLISH_INTERVAL)


def wait_enter(msg: str):
    try:
        input(msg)
    except KeyboardInterrupt:
        print("\n用户中断测试，正在尝试让所有关节回零...")
        zero_all_joints()
        sys.exit(0)


def zero_all_joints():
    for joint in JOINTS:
        publish_joint_position(joint["topic"], 0.0)
    print("[完成] 已发送所有关节回零指令。")


def main():
    print("======================================")
    print("TI5 机械臂逐关节交互测试")
    print("每次只测试一个关节")
    print("观察 Gazebo 中该关节是否正常运动")
    print("确认正常后，在本终端按 Enter 测试下一个关节")
    print("Ctrl + C 可随时中断并尝试回零")
    print("======================================\n")

    wait_enter("请确认无人机/机械臂状态稳定后，按 Enter 开始测试第 1 个关节...")

    # 先全部回零
    print("\n[初始化] 所有关节先回零")
    zero_all_joints()
    time.sleep(ZERO_WAIT)

    for idx, joint in enumerate(JOINTS, start=1):
        name = joint["name"]
        topic = joint["topic"]
        angle = joint["test_angle"]

        print("\n--------------------------------------")
        print(f"[测试 {idx}/6] 关节 {name}")
        print(f"控制话题: {topic}")
        print(f"目标角度: {angle} rad")
        print("--------------------------------------")

        # 发送测试角度
        print(f"[发送] 关节 {name} -> {angle} rad")
        publish_joint_position(topic, angle)

        # 等待用户观察确认
        wait_enter(f"请观察关节 {name} 是否正常活动。确认正常后按 Enter 回零并测试下一个关节...")

        # 当前关节回零
        print(f"[回零] 关节 {name} -> 0.0 rad")
        publish_joint_position(topic, 0.0)
        time.sleep(ZERO_WAIT)

    print("\n======================================")
    print("6 个关节已全部测试完成。")
    print("最终再次发送所有关节回零指令。")
    print("======================================")
    zero_all_joints()


if __name__ == "__main__":
    main()