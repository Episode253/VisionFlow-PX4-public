#!/bin/bash

# 平滑机械臂动作脚本
# 思路：两个姿态之间做余弦插值，而不是直接跳变
# 使用方法：
# chmod +x arm_smooth_show.sh
# ./arm_smooth_show.sh

send_joint() {
  topic=$1
  value=$2
  gz topic -t ${topic} -m gz.msgs.Double -p "data: ${value}" >/dev/null 2>&1
}

pose_all() {
  send_joint /joint/1/position_cmd $1
  send_joint /joint/2/position_cmd $2
  send_joint /joint/3/position_cmd $3
  send_joint /joint/4/position_cmd $4
  send_joint /joint/5/position_cmd $5
  send_joint /joint/6/position_cmd $6
}

smooth_move() {
  start_a=$1
  start_b=$2
  start_c=$3
  start_d=$4
  start_e=$5
  start_f=$6

  end_a=$7
  end_b=$8
  end_c=$9
  end_d=${10}
  end_e=${11}
  end_f=${12}

  duration=${13}
  steps=${14}

  for ((i=0; i<=steps; i++)); do
    values=$(python3 - <<EOF
import math

i = $i
steps = $steps

start = [$start_a, $start_b, $start_c, $start_d, $start_e, $start_f]
end   = [$end_a,   $end_b,   $end_c,   $end_d,   $end_e,   $end_f]

# 余弦插值：起步和结束都会比较柔和
s = 0.5 - 0.5 * math.cos(math.pi * i / steps)

out = []
for a, b in zip(start, end):
    out.append(a + (b - a) * s)

print(" ".join(f"{x:.5f}" for x in out))
EOF
)

    pose_all $values

    sleep_time=$(python3 - <<EOF
print($duration / $steps)
EOF
)
    sleep $sleep_time
  done
}

echo "机械臂平滑归零..."
pose_all 0.0 0.0 0.0 0.0 0.0 0.0
sleep 1.0

current_a=0.0
current_b=0.0
current_c=0.0
current_d=0.0
current_e=0.0
current_f=0.0

echo "动作1：平滑展开..."
smooth_move \
$current_a $current_b $current_c $current_d $current_e $current_f \
0.0 0.45 -0.65 0.0 0.45 0.0 \
2.5 60

current_a=0.0
current_b=0.45
current_c=-0.65
current_d=0.0
current_e=0.45
current_f=0.0

sleep 0.3

echo "动作2：柔和左右挥手..."
smooth_move \
$current_a $current_b $current_c $current_d $current_e $current_f \
-0.65 0.42 -0.70 0.25 0.48 0.0 \
1.2 36

smooth_move \
-0.65 0.42 -0.70 0.25 0.48 0.0 \
0.65 0.42 -0.70 -0.25 0.48 0.0 \
1.4 42

smooth_move \
0.65 0.42 -0.70 -0.25 0.48 0.0 \
-0.65 0.42 -0.70 0.25 0.48 0.0 \
1.4 42

smooth_move \
-0.65 0.42 -0.70 0.25 0.48 0.0 \
0.65 0.42 -0.70 -0.25 0.48 0.0 \
1.4 42

smooth_move \
0.65 0.42 -0.70 -0.25 0.48 0.0 \
0.0 0.42 -0.70 0.0 0.48 0.0 \
1.2 36

sleep 0.3

echo "动作3：腕部旋转展示..."
smooth_move \
0.0 0.42 -0.70 0.0 0.48 0.0 \
0.0 0.38 -0.65 0.0 0.55 -1.2 \
1.0 30

smooth_move \
0.0 0.38 -0.65 0.0 0.55 -1.2 \
0.0 0.38 -0.65 0.0 0.55 1.2 \
1.4 42

smooth_move \
0.0 0.38 -0.65 0.0 0.55 1.2 \
0.0 0.38 -0.65 0.0 0.55 -1.2 \
1.4 42

smooth_move \
0.0 0.38 -0.65 0.0 0.55 -1.2 \
0.0 0.38 -0.65 0.0 0.55 0.0 \
1.0 30

sleep 0.3

echo "动作4：空中礼花式展开..."
smooth_move \
0.0 0.38 -0.65 0.0 0.55 0.0 \
0.45 0.60 -0.95 0.55 0.70 0.8 \
1.5 45

smooth_move \
0.45 0.60 -0.95 0.55 0.70 0.8 \
-0.45 0.60 -0.95 -0.55 0.70 -0.8 \
1.5 45

smooth_move \
-0.45 0.60 -0.95 -0.55 0.70 -0.8 \
0.0 0.45 -0.70 0.0 0.45 0.0 \
1.5 45

sleep 0.5

echo "动作5：平滑收回..."
smooth_move \
0.0 0.45 -0.70 0.0 0.45 0.0 \
0.0 0.0 0.0 0.0 0.0 0.0 \
2.5 60

echo "机械臂平滑动作完成。"