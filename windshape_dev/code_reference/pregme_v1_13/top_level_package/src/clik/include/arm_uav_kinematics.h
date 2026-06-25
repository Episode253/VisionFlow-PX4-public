#ifndef ARM_UAV_KINEMATICS_H
#define ARM_UAV_KINEMATICS_H

#include <array>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "arm_uav_model.h"

namespace arm_uav_kinematics
{

using JointVector = Eigen::Matrix<double, arm_uav_model::kJointNum, 1>;

struct ComStateInBody
{
    Eigen::Vector3d p_c_uav_B = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_c_base_link_B = Eigen::Vector3d::Zero();
    std::array<Eigen::Vector3d, arm_uav_model::kArmLinkNum> p_c_links_B {};
    Eigen::Vector3d p_C_arm_B = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_C_B = Eigen::Vector3d::Zero();
    double m_arm = 0.0;
    double m_total = 0.0;
};

Eigen::Matrix4d makeDhTransform(const arm_uav_model::DhParam &dh, double q_i);

std::array<Eigen::Isometry3d, arm_uav_model::kArmLinkNum>
computeLinkPosesInArmBase(const JointVector &q,
                          const arm_uav_model::ArmUavParam &param = arm_uav_model::kModelParam);

ComStateInBody computeComStateInBody(
    const JointVector &q,
    const arm_uav_model::ArmUavParam &param = arm_uav_model::kModelParam);

Eigen::Vector3d computeArmComInBody(
    const JointVector &q,
    const arm_uav_model::ArmUavParam &param = arm_uav_model::kModelParam);

Eigen::Vector3d computeSystemComInBody(
    const JointVector &q,
    const arm_uav_model::ArmUavParam &param = arm_uav_model::kModelParam);

JointVector toJointVector(const Eigen::VectorXd &q);

ComStateInBody computeComStateInBody(
    const Eigen::VectorXd &q,
    const arm_uav_model::ArmUavParam &param = arm_uav_model::kModelParam);

Eigen::Vector3d computeArmComInBody(
    const Eigen::VectorXd &q,
    const arm_uav_model::ArmUavParam &param = arm_uav_model::kModelParam);

Eigen::Vector3d computeSystemComInBody(
    const Eigen::VectorXd &q,
    const arm_uav_model::ArmUavParam &param = arm_uav_model::kModelParam);

} // namespace arm_uav_kinematics

#endif // ARM_UAV_KINEMATICS_H
