#include "arm_uav_kinematics.h"

#include <stdexcept>

namespace arm_uav_kinematics
{

namespace
{

constexpr double kMassEps = 1e-9;

Eigen::Isometry3d makeIsometry(const Eigen::Matrix4d &T)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.matrix() = T;
    return pose;
}

//辅助函数，将机械臂基座坐标系里的一个点，变换到机体坐标系
Eigen::Vector3d transformPointToBody(const Eigen::Vector3d &p_in_arm_base,
                                     const arm_uav_model::ArmUavParam &param)
{
    return param.p_mount + param.R_mount * p_in_arm_base;
}

} // namespace

// 根据一节连杆的DH参数和关节角，得到该连杆相对于机械臂基座的DH变换矩阵
Eigen::Matrix4d makeDhTransform(const arm_uav_model::DhParam &dh, double q_i)
{
    const double theta_total = dh.theta + q_i;
    const double ct = std::cos(theta_total);
    const double st = std::sin(theta_total);
    const double ca = std::cos(dh.alpha);
    const double sa = std::sin(dh.alpha);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T << ct, -st * ca, st * sa, dh.a * ct,
         st, ct * ca, -ct * sa, dh.a * st,
         0.0, sa, ca, dh.d,
         0.0, 0.0, 0.0, 1.0;
    return T;
}

// 根据当前6个关节角，计算6个连杆A-F相对机械臂基座的位姿
std::array<Eigen::Isometry3d, arm_uav_model::kArmLinkNum>
computeLinkPosesInArmBase(const JointVector &q, const arm_uav_model::ArmUavParam &param)
{
    std::array<Eigen::Isometry3d, arm_uav_model::kArmLinkNum> link_poses {};
    Eigen::Matrix4d T_cumulative = Eigen::Matrix4d::Identity();

    for (int i = 0; i < arm_uav_model::kArmLinkNum; ++i) {
        T_cumulative = T_cumulative * makeDhTransform(param.dh[i], q(i));
        link_poses[i] = makeIsometry(T_cumulative);
    }

    // link_poses[i]是第i个连杆相对于机械臂基座的位姿，包含旋转和平移信息
    return link_poses;
}

// 核心函数，计算所有质心状态
ComStateInBody computeComStateInBody(const JointVector &q,
                                     const arm_uav_model::ArmUavParam &param)
{
    ComStateInBody state;
    state.p_c_uav_B = param.pc_uav;

    const auto link_poses_in_arm_base = computeLinkPosesInArmBase(q, param);

    // 机械臂基座的质心在机体坐标系下的位置
    state.p_c_base_link_B = transformPointToBody(param.base_link.pc, param);

    double arm_weight_sum = param.base_link.m;
    Eigen::Vector3d arm_moment_sum = param.base_link.m * state.p_c_base_link_B;

    // 遍历每个连杆，计算其质心在机体坐标系下的位置，并做质量加权累加
    for (int i = 0; i < arm_uav_model::kArmLinkNum; ++i) {
        const Eigen::Vector3d p_c_link_in_arm_base =
            link_poses_in_arm_base[i].translation()
            + link_poses_in_arm_base[i].rotation() * param.links[i].pc;

        state.p_c_links_B[i] = transformPointToBody(p_c_link_in_arm_base, param);

        arm_weight_sum += param.links[i].m;
        arm_moment_sum += param.links[i].m * state.p_c_links_B[i];
    }

    // 计算机械臂在机体系下的整体质心
    state.m_arm = arm_weight_sum;

    if (state.m_arm > kMassEps) {
        state.p_C_arm_B = arm_moment_sum / state.m_arm;
    } else {
        state.p_C_arm_B.setZero();
    }

    // 计算整个系统的质心
    state.m_total = param.m_uav + state.m_arm;

    if (state.m_total > kMassEps) {
        const Eigen::Vector3d total_moment_sum =
            param.m_uav * state.p_c_uav_B + arm_moment_sum;
        state.p_C_B = total_moment_sum / state.m_total;
    } else {
        state.p_C_B.setZero();
    }

    return state;
}

// 只返回机械臂质心在机体坐标系下的位置
Eigen::Vector3d computeArmComInBody(const JointVector &q,
                                    const arm_uav_model::ArmUavParam &param)
{
    return computeComStateInBody(q, param).p_C_arm_B;
}

// 只返回整个系统质心在机体坐标系下的位置
Eigen::Vector3d computeSystemComInBody(const JointVector &q,
                                       const arm_uav_model::ArmUavParam &param)
{
    return computeComStateInBody(q, param).p_C_B;
}

JointVector toJointVector(const Eigen::VectorXd &q)
{
    if (q.size() != arm_uav_model::kJointNum) {
        throw std::runtime_error("Joint vector size must equal arm_uav_model::kJointNum");
    }

    JointVector joint_q;
    for (int i = 0; i < arm_uav_model::kJointNum; ++i) {
        joint_q(i) = q(i);
    }
    return joint_q;
}

ComStateInBody computeComStateInBody(const Eigen::VectorXd &q,
                                     const arm_uav_model::ArmUavParam &param)
{
    return computeComStateInBody(toJointVector(q), param);
}

Eigen::Vector3d computeArmComInBody(const Eigen::VectorXd &q,
                                    const arm_uav_model::ArmUavParam &param)
{
    return computeArmComInBody(toJointVector(q), param);
}

Eigen::Vector3d computeSystemComInBody(const Eigen::VectorXd &q,
                                       const arm_uav_model::ArmUavParam &param)
{
    return computeSystemComInBody(toJointVector(q), param);
}

} // namespace arm_uav_kinematics
