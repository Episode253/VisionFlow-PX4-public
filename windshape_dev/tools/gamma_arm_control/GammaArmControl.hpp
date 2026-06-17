#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <map>

// KDL
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace gamma_arm
{

/// @brief GAMMA arm gravity compensation plugin for Gazebo Sim.
///
/// Implements PD + KDL gravity feedforward control for a 6-DOF arm.
/// Subscribes to /joint/gamma/N/position_cmd (gz::msgs::Double) per joint.
/// Computes gravity torques in real-time using KDL ChainDynParam.
///
/// Joint axes (from URDF):
///   A: (1,0,0)   B: (0,-1,0)  C: (0,-1,0)
///   D: (1,0,0)   E: (0,-1,0)  F: (0,0,1)
class GammaArmControlPlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
    GammaArmControlPlugin();
    ~GammaArmControlPlugin() override = default;

    // ISystemConfigure: called once during model load, allows reading SDF params
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr) override;

    // ISystemPreUpdate: called every simulation step before physics update
    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override;

private:
    /// Callback for receiving joint position commands via Gazebo transport
    void OnJointCmd(int joint_index, const gz::msgs::Double &_msg);

    /// Initialize KDL chain from ROS parameter server
    bool InitKDL();

    gz::sim::Model _model{gz::sim::kNullEntity};
    gz::transport::Node _node;

    /// Joint entities
    std::vector<gz::sim::Entity> _jointEntities;

    /// Joint names
    std::vector<std::string> _jointNames;

    /// Target positions for each joint (from cmd topic)
    std::vector<double> _targetPositions;

    /// PD gains
    std::vector<double> _pGains;
    std::vector<double> _dGains;

    /// Max joint efforts (torque limits)
    std::vector<double> _maxEfforts;

    /// Command topic prefix
    std::string _cmdTopicPrefix;

    /// KDL dynamics
    KDL::Chain _chain;
    std::unique_ptr<KDL::ChainDynParam> _dynParam;
    KDL::JntArray _q;
    KDL::JntArray _qDot;
    KDL::JntArray _gravityTorques;

    /// Gravity vector
    KDL::Vector _gravity;

    /// KDL chain root/tip link names
    std::string _kdlRoot;
    std::string _kdlTip;

    /// Mutex for target position access
    std::mutex _mutex;

    /// Whether KDL was initialized
    bool _kdlInitialized{false};

    /// Number of joints
    int _nJoints{0};

    /// Whether this is the first update (for initialization)
    bool _firstUpdate{true};
};

} // namespace gamma_arm
