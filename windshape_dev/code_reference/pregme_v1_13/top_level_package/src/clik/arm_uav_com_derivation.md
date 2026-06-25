# 机械臂 SDF 物理参数到 DH 质心参数、再到系统总质心的完整推导

本文把两层容易混在一起的工作拆开整理：

1. `GAMMA.sdf` 中的物理连杆惯性参数，如何逆推出标准 DH 局部坐标系下的质心坐标
2. 已得到的 DH 质心参数，如何在运行时映射到机体系并求整个机械臂-无人机系统总质心

对应代码与模型来源：

- [arm_uav_model.h](/home/an/catkin_ws/src/clik/include/arm_uav_model.h:14)
- [arm_uav_kinematics.h](/home/an/catkin_ws/src/clik/include/arm_uav_kinematics.h:16)
- [arm_uav_kinematics.cpp](/home/an/catkin_ws/src/clik/src/arm_uav_kinematics.cpp:20)
- [forward_kinematic.cpp](/home/an/catkin_ws/src/clik/src/forward_kinematic.cpp:13)
- `GAMMA.sdf`

需要先说明一个关键事实：

$$
\texttt{arm\_uav\_model.h}
$$

里保存的 `pc` 不是 SDF 原始惯性坐标，而是已经整理到各 DH 局部系下的结果。也就是说，你给出的那份流程，正是从 SDF 原始参数生成这些 `pc` 常量的前置推导过程。

## 1. 坐标系与符号约定

记号约定如下：

- `{S_i}`：SDF 中第 $i$ 个刚体或关节所依附的局部物理坐标系
- `{D_i}`：标准 DH 中第 $i$ 个局部坐标系
- `{A}`：机械臂基座坐标系，等价于 `{D_0}`
- `{B}`：无人机机体系

对任意点 $\mathbf{p}$，若其在坐标系 `{X}` 中表示，则写作

$$
{}^{X}\mathbf{p}
$$

若从 `{Y}` 到 `{X}` 的刚体变换为 $({}^{X}\mathbf{R}_{Y}, {}^{X}\mathbf{t}_{Y})$，则

$$
{}^{X}\mathbf{p} = {}^{X}\mathbf{R}_{Y}\,{}^{Y}\mathbf{p} + {}^{X}\mathbf{t}_{Y}
$$

对应齐次形式为

$$
{}^{X}\mathbf{T}_{Y} =
\begin{bmatrix}
{}^{X}\mathbf{R}_{Y} & {}^{X}\mathbf{t}_{Y} \\
\mathbf{0}^T & 1
\end{bmatrix}
$$

## 2. 整体任务分解

整个链路可以概括成下面两步：

### 2.1 线下辨识层：SDF 惯性参数映射到 DH 局部系

目标是把每个连杆在 SDF 中给出的惯性中心

$$
{}^{S_i}\mathbf{p}_{c,i}
$$

转换为代码里保存的 DH 局部质心参数

$$
{}^{D_i}\mathbf{p}_{c,i}
$$

### 2.2 运行时计算层：DH 局部质心映射到机体系并求总质心

目标是从

$$
\left\{ {}^{D_i}\mathbf{p}_{c,i} \right\}
$$

出发，结合关节角 $\mathbf{q}$，求得：

$$
{}^{B}\mathbf{p}_{C,arm}, \qquad {}^{B}\mathbf{p}_{C,total}
$$

这两层分别对应“模型参数构造”和“在线运动学求值”。

## 3. SDF 到 DH 的三条基本变换规律

你整理的流程本质上依赖以下三条规则，这部分非常关键。

### 3.1 主动装配变换

若已知子坐标系中的点 $ {}^{child}\mathbf{p} $，关节从子系到父系的安装旋转和平移分别为 $\mathbf{R}_{joint}$、$\mathbf{t}_{joint}$，则

$$
{}^{parent}\mathbf{p}
=
\mathbf{R}_{joint}\,{}^{child}\mathbf{p}
+
\mathbf{t}_{joint}
$$

这就是你文中写的“物理装配正向推导”。

### 3.2 坐标基准对齐的被动补偿

如果我们不是在移动刚体，而是在改写同一个物理点在另一个坐标基底下的表达，那么应使用逆基变换：

$$
{}^{new}\mathbf{p}
=
({}^{old}\mathbf{R}_{new})^T
\left(
{}^{old}\mathbf{p} - {}^{old}\mathbf{t}_{new}
\right)
$$

等价地，若已知从 `{new}` 到 `{old}` 的齐次变换 ${}^{old}\mathbf{T}_{new}$，则

$$
{}^{new}\tilde{\mathbf{p}}
=
\left({}^{old}\mathbf{T}_{new}\right)^{-1}
{}^{old}\tilde{\mathbf{p}}
$$

其中 $\tilde{\mathbf{p}}$ 是齐次坐标。

### 3.3 向后投影到下一级 DH 坐标系

若已经知道某点在 `{D_{i-1}}` 中的表达，想得到它在 `{D_i}` 中的表达，则

$$
{}^{D_i}\tilde{\mathbf{p}}
=
\left({}^{D_{i-1}}\mathbf{T}_{D_i}\right)^{-1}
{}^{D_{i-1}}\tilde{\mathbf{p}}
$$

这就是你写的“逆向 DH 补偿”。

## 4. 标准 DH 变换公式

代码使用的是标准 DH 形式。对第 $i$ 个关节，

$$
\theta_i^{tot} = \theta_i + q_i
$$

单级齐次变换为

$$
{}^{D_{i-1}}\mathbf{T}_{D_i}
=
\begin{bmatrix}
\cos\theta_i^{tot} & -\sin\theta_i^{tot}\cos\alpha_i & \sin\theta_i^{tot}\sin\alpha_i & a_i\cos\theta_i^{tot} \\
\sin\theta_i^{tot} & \cos\theta_i^{tot}\cos\alpha_i & -\cos\theta_i^{tot}\sin\alpha_i & a_i\sin\theta_i^{tot} \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

对应实现见 [arm_uav_kinematics.cpp](/home/an/catkin_ws/src/clik/src/arm_uav_kinematics.cpp:30)。

静态 DH 参数见 [arm_uav_model.h](/home/an/catkin_ws/src/clik/include/arm_uav_model.h:91)。

## 5. 从 SDF 物理参数到 DH 局部质心参数

这一节对应你整理的“逐级逆推记录”。

### 5.1 Link 与 DH 坐标系映射

在当前整理结果中：

$$
\texttt{base\_link} \rightarrow D_0
$$
$$
\texttt{A\_Link} \rightarrow D_1
$$
$$
\texttt{B\_Link} \rightarrow D_2
$$
$$
\texttt{C\_Link} \rightarrow D_3
$$
$$
\texttt{D\_Link} \rightarrow D_4
$$
$$
\texttt{E\_Link},\texttt{F\_Link} \rightarrow D_5
$$

最后一条要特别说明：当前代码中并没有保留 SDF 原始的

$$
m_E = 0.042397,\qquad m_F = 0.192
$$

而是直接将二者在 $D_5$ 下合成为

$$
m_{EF} = 0.234397 \approx 0.234
$$

写入了 [arm_uav_model.h](/home/an/catkin_ws/src/clik/include/arm_uav_model.h:174)，同时把 `F_Link` 的质量置为 0，见 [arm_uav_model.h](/home/an/catkin_ws/src/clik/include/arm_uav_model.h:181)。

### 5.2 第 0 层：SDF 基准对齐到 $D_0$

根据你的整理，SDF 中基准安装位相对 $D_0$ 的关系可概括为：

$$
{}^{S_0}\mathbf{t}_{D_0} =
\begin{bmatrix}
-0.0613 \\ 0 \\ 0
\end{bmatrix}
$$

并伴随一个绕 $Y$ 轴的基准对齐旋转。若采用你文中的表述，则点坐标补偿等价于

$$
{}^{D_0}\mathbf{p}
=
\mathbf{R}_y(-\pi/2)
\left(
{}^{S_0}\mathbf{p}
-
{}^{S_0}\mathbf{t}_{D_0}
\right)
$$

对 `base_link`，SDF 给出的惯性中心是

$$
{}^{S_0}\mathbf{p}_{c,base}
=
\begin{bmatrix}
0.026 \\ 0 \\ 0
\end{bmatrix}
$$

整理后得到

$$
{}^{D_0}\mathbf{p}_{c,base}
=
\begin{bmatrix}
0 \\ 0 \\ 0.026
\end{bmatrix}
$$

这正对应代码中的

$$
\texttt{param.base\_link.pc} =
\begin{bmatrix}
0 \\ 0 \\ 0.026
\end{bmatrix}
$$

见 [arm_uav_model.h](/home/an/catkin_ws/src/clik/include/arm_uav_model.h:139)。

### 5.3 第 1 层：A_Link 映射到 $D_1$

SDF 惯性中心为

$$
{}^{S_A}\mathbf{p}_{c,A}
=
\begin{bmatrix}
0.036314 \\ 0.007409 \\ 0.053527
\end{bmatrix}
$$

先对齐到 $D_0$：

$$
{}^{D_0}\mathbf{p}_{c,A}
=
\begin{bmatrix}
-0.053527 \\ 0.007409 \\ 0.097614
\end{bmatrix}
$$

再通过

$$
{}^{D_1}\tilde{\mathbf{p}}_{c,A}
=
\left({}^{D_0}\mathbf{T}_{D_1}\right)^{-1}
{}^{D_0}\tilde{\mathbf{p}}_{c,A}
$$

得到

$$
{}^{D_1}\mathbf{p}_{c,A}
\approx
\begin{bmatrix}
-0.010543 \\ 0.006676 \\ -0.007409
\end{bmatrix}
$$

与代码中的

$$
\begin{bmatrix}
-0.0105 \\ 0.0067 \\ -0.0074
\end{bmatrix}
$$

一致，见 [arm_uav_model.h](/home/an/catkin_ws/src/clik/include/arm_uav_model.h:146)。

### 5.4 第 2 层：B_Link 映射到 $D_2$

SDF 中：

$$
{}^{S_B}\mathbf{p}_{c,B}
=
\begin{bmatrix}
-0.14539 \\ -0.024762 \\ 0.16247
\end{bmatrix}
$$

先通过 Joint B 安装变换推回 A_Link 物理系：

$$
{}^{S_A}\mathbf{p}_{c,B}
=
\mathbf{R}_y(0.72990)\,{}^{S_B}\mathbf{p}_{c,B}
+
\begin{bmatrix}
0.042986 \\ 0.03395 \\ 0.064067
\end{bmatrix}
$$

再一路投影到 $D_1$，你得到

$$
{}^{D_1}\mathbf{p}_{c,B}
\approx
\begin{bmatrix}
0.218017 \\ 0.000014 \\ -0.009188
\end{bmatrix}
$$

最后用

$$
{}^{D_2}\tilde{\mathbf{p}}_{c,B}
=
\left({}^{D_1}\mathbf{T}_{D_2}\right)^{-1}
{}^{D_1}\tilde{\mathbf{p}}_{c,B}
$$

得到

$$
{}^{D_2}\mathbf{p}_{c,B}
\approx
\begin{bmatrix}
-0.030713 \\ 0 \\ -0.032238
\end{bmatrix}
$$

与代码中的

$$
\begin{bmatrix}
-0.0307 \\ 0 \\ -0.0322
\end{bmatrix}
$$

一致，见 [arm_uav_model.h](/home/an/catkin_ws/src/clik/include/arm_uav_model.h:153)。

### 5.5 第 3 层：C_Link 映射到 $D_3$

SDF 惯性中心为

$$
{}^{S_C}\mathbf{p}_{c,C}
=
\begin{bmatrix}
0.001741 \\ 0.023334 \\ 0.057853
\end{bmatrix}
$$

经过

$$
S_C \rightarrow S_B \rightarrow S_A \rightarrow D_0 \rightarrow D_1 \rightarrow D_2
$$

的逐级逆推后，你得到

$$
{}^{D_2}\mathbf{p}_{c,C}
\approx
\begin{bmatrix}
0.057511 \\ 0.006572 \\ -0.023334
\end{bmatrix}
$$

再做

$$
{}^{D_3}\tilde{\mathbf{p}}_{c,C}
=
\left({}^{D_2}\mathbf{T}_{D_3}\right)^{-1}
{}^{D_2}\tilde{\mathbf{p}}_{c,C}
$$

得到

$$
{}^{D_3}\mathbf{p}_{c,C}
\approx
\begin{bmatrix}
-0.005499 \\ 0.001666 \\ -0.006572
\end{bmatrix}
$$

对应代码见 [arm_uav_model.h](/home/an/catkin_ws/src/clik/include/arm_uav_model.h:160)。

### 5.6 第 4 层：D_Link 映射到 $D_4$

SDF 惯性中心为

$$
{}^{S_D}\mathbf{p}_{c,D}
=
\begin{bmatrix}
0.13241 \\ -0.004409 \\ 0.000076
\end{bmatrix}
$$

逐级逆推到 $D_3$ 后，你得到

$$
{}^{D_3}\mathbf{p}_{c,D}
\approx
\begin{bmatrix}
0.004347 \\ -0.000076 \\ 0.147422
\end{bmatrix}
$$

再利用

$$
{}^{D_4}\tilde{\mathbf{p}}_{c,D}
=
\left({}^{D_3}\mathbf{T}_{D_4}\right)^{-1}
{}^{D_3}\tilde{\mathbf{p}}_{c,D}
$$

得到

$$
{}^{D_4}\mathbf{p}_{c,D}
\approx
\begin{bmatrix}
-0.000076 \\ -0.017578 \\ 0.004347
\end{bmatrix}
$$

与代码中的

$$
\begin{bmatrix}
-7.6 \times 10^{-5} \\ -0.0176 \\ 0.0043
\end{bmatrix}
$$

一致，见 [arm_uav_model.h](/home/an/catkin_ws/src/clik/include/arm_uav_model.h:167)。

### 5.7 第 5 层：E_Link 与 F_Link 合成到 $D_5$

这里是最值得单独说明的一步。

SDF 原始质量为

$$
m_E = 0.042397,\qquad m_F = 0.192
$$

总质量为

$$
m_{EF} = m_E + m_F = 0.234397
$$

你分别求得：

$$
{}^{D_5}\mathbf{p}_{c,E}
\approx
\begin{bmatrix}
-0.000002 \\ 0.020111 \\ 0.051627
\end{bmatrix}
$$

$$
{}^{D_5}\mathbf{p}_{c,F}
\approx
\begin{bmatrix}
0.000046 \\ -0.000204 \\ 0.059614
\end{bmatrix}
$$

因此合成后的等效质心为

$$
{}^{D_5}\mathbf{p}_{c,EF}
=
\frac{
m_E\,{}^{D_5}\mathbf{p}_{c,E}
+
m_F\,{}^{D_5}\mathbf{p}_{c,F}
}{
m_E + m_F
}
$$

即

$$
{}^{D_5}\mathbf{p}_{c,EF}
\approx
\begin{bmatrix}
0.000037 \\ 0.003470 \\ 0.058169
\end{bmatrix}
$$

这与代码中 `links[4]` 的

$$
\begin{bmatrix}
3.7\times 10^{-5} \\ 0.0035 \\ 0.0582
\end{bmatrix}
$$

一致，见 [arm_uav_model.h](/home/an/catkin_ws/src/clik/include/arm_uav_model.h:174)。

而 `links[5]` 虽然保留了 `F_Link` 的局部坐标，但质量被设为

$$
m_{F,\text{runtime}} = 0
$$

所以运行时总质心计算中，真正起作用的是这个已经合成后的 `E/F` 等效刚体。

## 6. 最终得到的 DH 局部质心参数

整理后的局部质心参数可以写成：

$$
{}^{D_0}\mathbf{p}_{c,base}
=
\begin{bmatrix}
0 \\ 0 \\ 0.026000
\end{bmatrix}
$$

$$
{}^{D_1}\mathbf{p}_{c,A}
\approx
\begin{bmatrix}
-0.010543 \\ 0.006676 \\ -0.007409
\end{bmatrix}
$$

$$
{}^{D_2}\mathbf{p}_{c,B}
\approx
\begin{bmatrix}
-0.030713 \\ 0 \\ -0.032238
\end{bmatrix}
$$

$$
{}^{D_3}\mathbf{p}_{c,C}
\approx
\begin{bmatrix}
-0.005499 \\ 0.001666 \\ -0.006572
\end{bmatrix}
$$

$$
{}^{D_4}\mathbf{p}_{c,D}
\approx
\begin{bmatrix}
-0.000076 \\ -0.017578 \\ 0.004347
\end{bmatrix}
$$

$$
{}^{D_5}\mathbf{p}_{c,EF}
\approx
\begin{bmatrix}
0.000037 \\ 0.003470 \\ 0.058169
\end{bmatrix}
$$

这些量就是 `arm_uav_model.h` 中各 `pc` 参数的来源。

## 7. 运行时：从 DH 局部质心到机械臂各刚体位姿

在运行时，输入关节角

$$
\mathbf{q} = [q_1,q_2,q_3,q_4,q_5,q_6]^T
$$

代码先构造每一级 DH 变换

$$
{}^{D_{i-1}}\mathbf{T}_{D_i}(\mathbf{q})
$$

然后累乘得到第 $i$ 个连杆局部系相对机械臂基座 `{A}={D_0}` 的位姿：

$$
{}^{A}\mathbf{T}_{D_i}
=
\prod_{k=1}^{i}
{}^{D_{k-1}}\mathbf{T}_{D_k}
$$

这正是 [arm_uav_kinematics.cpp](/home/an/catkin_ws/src/clik/src/arm_uav_kinematics.cpp:47) 中 `computeLinkPosesInArmBase(...)` 的核心。

若将

$$
{}^{A}\mathbf{T}_{D_i}
=
\begin{bmatrix}
{}^{A}\mathbf{R}_{D_i} & {}^{A}\mathbf{t}_{D_i} \\
\mathbf{0}^T & 1
\end{bmatrix}
$$

拆开，则第 $i$ 个活动连杆质心在 `{A}` 中的位置为

$$
{}^{A}\mathbf{p}_{c,i}
=
{}^{A}\mathbf{t}_{D_i}
+
{}^{A}\mathbf{R}_{D_i}\,{}^{D_i}\mathbf{p}_{c,i}
$$

对应代码：

$$
\texttt{translation()} + \texttt{rotation()} \cdot \texttt{pc}
$$

见 [arm_uav_kinematics.cpp](/home/an/catkin_ws/src/clik/src/arm_uav_kinematics.cpp:79)。

## 8. 从机械臂基座系到无人机机体系

代码里机械臂安装位姿定义为

$$
{}^{B}\mathbf{t}_{A}
=
\begin{bmatrix}
0.398 \\ 0 \\ 0
\end{bmatrix}
$$

$$
{}^{B}\mathbf{R}_{A}
=
\begin{bmatrix}
0 & 0 & 1 \\
0 & 1 & 0 \\
-1 & 0 & 0
\end{bmatrix}
$$

见 [arm_uav_model.h](/home/an/catkin_ws/src/clik/include/arm_uav_model.h:130)。

因此任意一个在 `{A}` 中的点变换到机体系 `{B}` 的公式为

$$
{}^{B}\mathbf{p}
=
{}^{B}\mathbf{t}_{A}
+
{}^{B}\mathbf{R}_{A}\,{}^{A}\mathbf{p}
$$

展开可得

$$
{}^{B}\mathbf{p}
=
\begin{bmatrix}
0.398 + z_A \\
y_A \\
-x_A
\end{bmatrix}
$$

这就是 `transformPointToBody(...)` 的数学表达，见 [arm_uav_kinematics.cpp](/home/an/catkin_ws/src/clik/src/arm_uav_kinematics.cpp:21)。

特别地：

$$
{}^{B}\mathbf{p}_{c,base}
=
{}^{B}\mathbf{t}_{A}
+
{}^{B}\mathbf{R}_{A}\,{}^{D_0}\mathbf{p}_{c,base}
$$

$$
{}^{B}\mathbf{p}_{c,i}
=
{}^{B}\mathbf{t}_{A}
+
{}^{B}\mathbf{R}_{A}\,{}^{A}\mathbf{p}_{c,i}
$$

而无人机本体质心本来就定义在机体系中：

$$
{}^{B}\mathbf{p}_{c,uav} = \mathbf{pc}_{uav}
$$

## 9. 机械臂整体质心与系统总质心

### 9.1 机械臂整体质心

设机械臂部分总质量为

$$
m_{arm} = m_{base} + \sum_{i=1}^{6} m_i
$$

则机械臂整体质心为

$$
{}^{B}\mathbf{p}_{C,arm}
=
\frac{
m_{base}\,{}^{B}\mathbf{p}_{c,base}
+
\sum_{i=1}^{6} m_i\,{}^{B}\mathbf{p}_{c,i}
}{
m_{arm}
}
$$

代码里先计算机械臂质量矩

$$
\mathbf{h}_{arm}
=
m_{base}\,{}^{B}\mathbf{p}_{c,base}
+
\sum_{i=1}^{6} m_i\,{}^{B}\mathbf{p}_{c,i}
$$

再令

$$
{}^{B}\mathbf{p}_{C,arm}
=
\frac{\mathbf{h}_{arm}}{m_{arm}}
$$

对应 [arm_uav_kinematics.cpp](/home/an/catkin_ws/src/clik/src/arm_uav_kinematics.cpp:74)。

### 9.2 系统总质心

设系统总质量为

$$
m_{total} = m_{uav} + m_{arm}
$$

则整个机械臂-无人机系统的总质心为

$$
{}^{B}\mathbf{p}_{C,total}
=
\frac{
m_{uav}\,{}^{B}\mathbf{p}_{c,uav}
+
\mathbf{h}_{arm}
}{
m_{total}
}
$$

展开后即

$$
{}^{B}\mathbf{p}_{C,total}
=
\frac{
m_{uav}\,{}^{B}\mathbf{p}_{c,uav}
+
m_{base}\,{}^{B}\mathbf{p}_{c,base}
+
\sum_{i=1}^{6} m_i\,{}^{B}\mathbf{p}_{c,i}
}{
m_{uav} + m_{base} + \sum_{i=1}^{6} m_i
}
$$

对应实现见 [arm_uav_kinematics.cpp](/home/an/catkin_ws/src/clik/src/arm_uav_kinematics.cpp:98)。

## 10. 这份流程和当前代码实现的关系

这部分值得明确写清，否则后面很容易混淆。

### 10.1 你的流程解决的是“参数来源”问题

你整理的这份链路说明了：

$$
\text{SDF 惯性中心}
\longrightarrow
\text{DH 局部质心参数}
$$

也就是 `arm_uav_model.h` 中这些常量为什么是现在这个数值。

### 10.2 当前代码解决的是“在线求值”问题

`arm_uav_kinematics.cpp` 做的是：

$$
\text{DH 局部质心参数}
\longrightarrow
\text{当前关节角下的机体系质心}
\longrightarrow
\text{机械臂质心与系统总质心}
$$

### 10.3 E/F 在 SDF 层和代码层并不完全同构

SDF 层是

$$
E\_Link,\ F\_Link
$$

两个独立刚体；而当前代码层等效成了

$$
(E+F)\ \text{in}\ D_5
$$

再加一个质量为 0 的 `F_Link` 占位项。这个设计在做质心计算时没有问题，但若后续要做更精细的惯量递推、柔顺控制或末端碰撞建模，就需要先确认是否继续沿用这个“末端合并刚体”的简化。

## 11. 一句话总结

整条链路可以压缩成一句话：

$$
\text{先把 SDF 中每个刚体的惯性中心逆推到对应 DH 局部系，得到 } {}^{D_i}\mathbf{p}_{c,i},
$$

$$
\text{再在运行时利用 DH 正运动学把这些局部质心映射到机体系，最后用质量加权平均得到机械臂和整机总质心。}
$$

### 9.3 `transformPointToBody(...)`

作用：

- 把机械臂基座系中的点变换到无人机机体系

公式：

```math
{}^B\mathbf{p} = {}^B\mathbf{p}_{mount} + {}^B\mathbf{R}_A {}^A\mathbf{p}
```

对应位置：

- [arm_uav_kinematics.cpp](/home/an/catkin_ws/src/clik/src/arm_uav_kinematics.cpp:21)

### 9.4 `computeComStateInBody(...)`

作用：

- 一次性算出无人机质心、机械臂基座质心、各连杆质心、机械臂整体质心、系统总质心

输出结构体：

- `p_c_uav_B`
- `p_c_base_link_B`
- `p_c_links_B`
- `p_C_arm_B`
- `p_C_B`
- `m_arm`
- `m_total`

定义见 [arm_uav_kinematics.h](/home/an/catkin_ws/src/clik/include/arm_uav_kinematics.h:16)，实现见 [arm_uav_kinematics.cpp](/home/an/catkin_ws/src/clik/src/arm_uav_kinematics.cpp:63)。

## 10. 实现层面的关键理解

### 10.1 当前质心计算的最终参考系是机体系 `{B}`

不管中间做了多少级 DH 累乘，最终所有质心都被统一变换到无人机机体系 `{B}` 中，再进行质量加权。这样做的好处是：

- 便于和飞控/机体动力学统一
- 便于后续做系统质心补偿、配平和控制分配

### 10.2 `links[i].pc` 不是世界系坐标，也不是机体系坐标

它表示的是：

```text
第 i 个连杆质心在该连杆对应 DH 局部坐标系中的位置
```

所以在求系统质心前，必须先经过：

1. 连杆局部系 -> 机械臂基座系
2. 机械臂基座系 -> 机体系

### 10.3 `F_Link` 当前质量为 0

在默认参数中：

```cpp
param.links[5].m = 0.0;
```

见 [arm_uav_model.h](/home/an/catkin_ws/src/clik/include/arm_uav_model.h:181)。

这意味着：

- `F_Link` 的位置仍然会被计算
- 但它对机械臂总质量和系统总质心没有贡献

## 11. 一句话总结

这套实现的本质就是：

先用 DH 参数把每个连杆的局部质心从“各自局部坐标系”推到“机械臂基座坐标系”，再通过安装位姿推到“无人机机体系”，最后用质量加权平均分别得到机械臂整体质心和整个机械臂-无人机系统的总质心。
