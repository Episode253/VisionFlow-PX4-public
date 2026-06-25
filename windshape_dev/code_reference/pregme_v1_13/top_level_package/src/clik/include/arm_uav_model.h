#ifndef ARM_UAV_MODEL_H
#define ARM_UAV_MODEL_H

#include <array>
#include <Eigen/Dense>

namespace arm_uav_model
{
// 机械臂活动关节数
constexpr int kJointNum = 6;
// 机械臂活动连杆数: A_Link ~ F_Link
constexpr int kArmLinkNum = 6;

// 单个关节的 DH 参数, 顺序为 base_link -> A_Link -> ... -> F_Link
struct DhParam
{
    // 连杆长度: 当前坐标轴到下一个坐标轴沿 x 方向的距离
    double a = 0.0;
    // 连杆扭转角: 两个相邻关节轴之间沿 x 轴旋转的夹角
    double alpha = 0.0;
    // 连杆偏距: 当前坐标轴到下一个坐标轴沿 z 方向的距离
    double d = 0.0;
    // 关节角偏置: 当前坐标轴到下一个坐标轴沿 z 轴旋转的夹角
    double theta = 0.0;
};

// 单个刚体(link)的动力学参数
struct BodyParam
{
    // 刚体质量
    double m = 0.0;
    // 刚体质心在对应 DH 局部系下的位置
    // base_link 对应 DH{0}, A_Link~F_Link 分别对应 DH{1}~DH{6}
    Eigen::Vector3d pc = Eigen::Vector3d::Zero();
    // 刚体相对于自身质心的惯量矩阵
    Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
};

// 机械臂-无人机模型参数
// 当前按 GAMMA.sdf 组织:
// 1. base_link 为机械臂基座刚体
// 2. A_Link ~ F_Link 为 6 个活动连杆
// 3. dh[0] ~ dh[5] 为从 base_link 到末端的 6 组 DH 参数
struct ArmUavParam
{
    // 无人机本体质量
    double m_uav = 0.0;
    // 无人机本体质心相对机体系原点的位置
    Eigen::Vector3d pc_uav = Eigen::Vector3d::Zero();
    // 无人机本体相对自身质心的等效惯量矩阵
    Eigen::Matrix3d I_uav = Eigen::Matrix3d::Zero();
    // 机械臂基座相对无人机机体系的安装位置
    Eigen::Vector3d p_mount = Eigen::Vector3d::Zero();
    // 机械臂基座相对无人机机体系的安装旋转
    Eigen::Matrix3d R_mount = Eigen::Matrix3d::Identity();
    // 机械臂基座刚体: base_link
    BodyParam base_link {};
    // 机械臂活动连杆: links[0]~links[5] 对应 A_Link~F_Link
    std::array<BodyParam, kArmLinkNum> links {};
    std::array<DhParam, kJointNum> dh {};
};

// 参数整体初始化
inline ArmUavParam makeDefaultParam()
{
    constexpr double kPi = 3.14159265358979323846;

    ArmUavParam param;
    param.m_uav = 0.0;
    param.pc_uav.setZero();
    param.I_uav.setZero();
    param.p_mount.setZero();
    param.R_mount.setIdentity();
    param.base_link.m = 0.0;
    param.base_link.pc.setZero();
    param.base_link.I.setZero();

    for (auto &link : param.links) {
        link.m = 0.0;
        link.pc.setZero();
        link.I.setZero();
    }

    for (auto &joint : param.dh) {
        joint.a = 0.0;
        joint.alpha = 0.0;
        joint.d = 0.0;
        joint.theta = 0.0;
    }

    // DH 参数: base_link -> A_Link -> B_Link -> C_Link -> D_Link -> E_Link -> F_Link
    
    param.dh[0].a = 0.06407;
    param.dh[0].alpha = -kPi / 2.0;
    param.dh[0].d = 0.10429;
    param.dh[0].theta = kPi;

    param.dh[1].a = 0.24873;
    param.dh[1].alpha = 0.0;
    param.dh[1].d = 0.02305;
    param.dh[1].theta = 0.0;

    param.dh[2].a = 0.06301;
    param.dh[2].alpha = kPi / 2.0;
    param.dh[2].d = -0.025;
    param.dh[2].theta = 0.0;

    param.dh[3].a = 0.0;
    param.dh[3].alpha = kPi / 2.0;
    param.dh[3].d = 0.165;
    param.dh[3].theta = kPi / 2.0;

    param.dh[4].a = 0.0;
    param.dh[4].alpha = kPi / 2.0;
    param.dh[4].d = -0.0015;
    param.dh[4].theta = kPi;

    param.dh[5].a = 0.0;
    param.dh[5].alpha = 0.0;
    param.dh[5].d = 0.084;
    param.dh[5].theta = kPi;

    // 无人机参数
    param.m_uav = 8.0108;
    param.pc_uav << -0.184037544, -1.24447246e-05, -0.0136405228;
    param.I_uav << 0.2773650044, -6.60955e-05, -0.0341333898,
                   -6.60955e-05, 0.70531458, 6.77452e-05,
                   -0.0341333898, 6.77452e-05, 0.9189226166;

    // 机械臂安装位姿
    // 机械臂基座系: x上 y右 z前
    // PX4机体系: x前 y右 z下
    // 这里定义的是: arm_base -> PX4 body
    param.p_mount << 0.398, 0.0, 0.0;
    param.R_mount << 0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0,
                    -1.0, 0.0, 0.0;

    // base_link
    param.base_link.m = 0.361;
    param.base_link.pc << 0.0, 0.0, 0.026;
    param.base_link.I << 0.00014566, 0.0, 0.0,
                         0.0, 0.00015872, 0.0,
                         0.0, 0.0, 0.00015872;

    // A_Link
    param.links[0].m = 0.482;
    param.links[0].pc << -0.0105, 0.0067, -0.0074;
    param.links[0].I << 0.00048916, -3.127e-05, 0.00014844,
                        -3.127e-05, 0.00049299, -5.578e-05,
                        0.00014844, -5.578e-05, 0.0003462;

    // B_Link
    param.links[1].m = 0.481;
    param.links[1].pc << -0.0307, 0.0, -0.0322;
    param.links[1].I << 0.0015459, 0.00021428, -0.0011607,
                        0.00021428, 0.0025514, -0.00023947,
                        -0.0011607, -0.00023947, 0.0012874;

    // C_Link
    param.links[2].m = 0.239;
    param.links[2].pc << -0.0055, 0.0017, -0.0066;
    param.links[2].I << 0.00014499, -3.1e-07, -4.26e-06,
                        -3.1e-07, 0.0001552, 1.928e-05,
                        -4.26e-06, 1.928e-05, 0.00010724;

    // D_Link
    param.links[3].m = 0.262;
    param.links[3].pc << -7.6e-05, -0.0176, 0.0043;
    param.links[3].I << 0.00010245, -1.568e-05, 3.8e-07,
                        -1.568e-05, 0.00052076, 3.2e-07,
                        3.8e-07, 3.2e-07, 0.0005369;

    // E_Link
    param.links[4].m = 0.234;
    param.links[4].pc << 3.7e-05, 0.0035, 0.0582;
    param.links[4].I << 6.544e-05, 0.0, 0.0,
                        0.0, 5.714e-05, 1.564e-05,
                        0.0, 1.564e-05, 2.005e-05;

    // F_Link
    param.links[5].m = 0.0;
    param.links[5].pc << -4.6e-05, 0.000103, -0.024394;
    param.links[5].I << 8.297e-05, 3e-08, 1.6e-07,
                        3e-08, 8.274e-05, -3.1e-07,
                        1.6e-07, -3.1e-07, 6.854e-05;

    return param;
}

inline const ArmUavParam kModelParam = makeDefaultParam();

} // namespace arm_uav_model

#endif // ARM_UAV_MODEL_H
