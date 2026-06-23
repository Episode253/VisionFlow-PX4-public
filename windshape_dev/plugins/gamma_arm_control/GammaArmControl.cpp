#include "GammaArmControl.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/Name.hh>

#include <gz/common/Console.hh>

#include <sdf/Element.hh>

#include <functional>
#include <chrono>
#include <fstream>
#include <regex>
#include <thread>

namespace gamma_arm
{

GammaArmControlPlugin::GammaArmControlPlugin()
{
    // nothing
}

void GammaArmControlPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
    _model = gz::sim::Model(_entity);

    if (!_model.Valid(_ecm)) {
        gzerr << "GammaArmControlPlugin: Model is invalid!" << std::endl;
        return;
    }

    // Read joint names from SDF
    if (_sdf->HasElement("joint_name")) {
        auto elem = _sdf->FindElement("joint_name");
        while (elem) {
            std::string name = elem->Get<std::string>();
            if (!name.empty()) {
                _jointNames.push_back(name);
            }
            elem = elem->GetNextElement("joint_name");
        }
    }

    if (_jointNames.empty()) {
        gzerr << "GammaArmControlPlugin: No joint_name elements found!" << std::endl;
        return;
    }

    _nJoints = static_cast<int>(_jointNames.size());

    // Resize vectors
    _jointEntities.resize(_nJoints, gz::sim::kNullEntity);
    _targetPositions.resize(_nJoints, 0.0);
    _initialPositions.resize(_nJoints, 0.0);
    _hasInitialPosition.resize(_nJoints, false);
    _pGains.resize(_nJoints, 10.0);
    _dGains.resize(_nJoints, 0.1);
    _maxEfforts.resize(_nJoints, 100.0);
    _q.resize(static_cast<unsigned int>(_nJoints));
    _qDot.resize(static_cast<unsigned int>(_nJoints));
    _gravityTorques.resize(static_cast<unsigned int>(_nJoints));

    // Look up joint entities by name
    for (int i = 0; i < _nJoints; ++i) {
        auto jointEntity = _model.JointByName(_ecm, _jointNames[i]);
        if (jointEntity == gz::sim::kNullEntity) {
            // Try with the nested model prefix (e.g. "gamma_arm::A")
            std::string nestedName = "gamma_arm::" + _jointNames[i];
            jointEntity = _model.JointByName(_ecm, nestedName);
        }
        if (jointEntity == gz::sim::kNullEntity) {
            gzerr << "GammaArmControlPlugin: Joint '" << _jointNames[i]
                  << "' not found in model!" << std::endl;
            return;
        }
        _jointEntities[i] = jointEntity;
        gzmsg << "GammaArmControlPlugin: Found joint: " << _jointNames[i] << std::endl;
    }

    // Read PID gains from SDF
    for (int i = 0; i < _nJoints; ++i) {
        std::string pKey = "p_gain_" + _jointNames[i];
        std::string dKey = "d_gain_" + _jointNames[i];
        std::string effortKey = "max_effort_" + _jointNames[i];

        if (_sdf->HasElement(pKey)) {
            _pGains[i] = _sdf->Get<double>(pKey, _pGains[i]).first;
        }
        if (_sdf->HasElement(dKey)) {
            _dGains[i] = _sdf->Get<double>(dKey, _dGains[i]).first;
        }
        if (_sdf->HasElement(effortKey)) {
            _maxEfforts[i] = _sdf->Get<double>(effortKey, _maxEfforts[i]).first;
        }
    }

    ReadInitialPositionsFromSDF(_sdf);

    // Read cmd topic prefix
    _cmdTopicPrefix = "/joint/gamma";
    if (_sdf->HasElement("cmd_topic_prefix")) {
        _cmdTopicPrefix = _sdf->Get<std::string>("cmd_topic_prefix", _cmdTopicPrefix).first;
    }

    // Subscribe to position command topics
    for (int i = 0; i < _nJoints; ++i) {
        std::string topic = _cmdTopicPrefix + "/" + std::to_string(i + 1) + "/position_cmd";

        // Must use std::function to help template deduction with lambda captures
        std::function<void(const gz::msgs::Double &)> cb =
            [this, i](const gz::msgs::Double &msg) {
                this->OnJointCmd(i, msg);
            };

        if (!_node.Subscribe(topic, cb)) {
            gzerr << "GammaArmControlPlugin: Failed to subscribe to " << topic << std::endl;
        } else {
            gzmsg << "GammaArmControlPlugin: Subscribed to " << topic << std::endl;
        }
    }

    // Read KDL parameters
    _kdlRoot = "dummy_root_link";
    _kdlTip = "F_Link";
    if (_sdf->HasElement("kdl_root")) {
        _kdlRoot = _sdf->Get<std::string>("kdl_root", _kdlRoot).first;
    }
    if (_sdf->HasElement("kdl_tip")) {
        _kdlTip = _sdf->Get<std::string>("kdl_tip", _kdlTip).first;
    }

    // Gravity vector
    double gx = 0.0, gy = 0.0, gz = 9.81;
    if (_sdf->HasElement("gravity_x")) {
        gx = _sdf->Get<double>("gravity_x", gx).first;
    }
    if (_sdf->HasElement("gravity_y")) {
        gy = _sdf->Get<double>("gravity_y", gy).first;
    }
    if (_sdf->HasElement("gravity_z")) {
        gz = _sdf->Get<double>("gravity_z", gz).first;
    }
    _gravity = KDL::Vector(gx, gy, gz);

    // Initialize KDL (will retry if /robot_description not yet available)
    // Give ROS param server a moment
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    _kdlInitialized = InitKDL();

    gzmsg << "GammaArmControlPlugin configured: " << _nJoints << " joints, "
          << "KDL initialized: " << (_kdlInitialized ? "yes" : "no") << std::endl;
}


void GammaArmControlPlugin::ReadInitialPositionsFromSDF(
    const std::shared_ptr<const sdf::Element> &_sdf)
{
    _hasAnyInitialPosition = false;

    for (int i = 0; i < _nJoints; ++i) {
        const std::string &jointName = _jointNames[i];

        const std::string nameKey = "initial_position_" + jointName;

        const std::string indexKey = "initial_position_" + std::to_string(i + 1);

        double value = 0.0;
        bool found = false;

        if (_sdf->HasElement(nameKey)) {
            value = _sdf->Get<double>(nameKey, value).first;
            found = true;
        } else if (_sdf->HasElement(indexKey)) {
            value = _sdf->Get<double>(indexKey, value).first;
            found = true;
        }

        if (found) {
            _initialPositions[i] = value;
            _hasInitialPosition[i] = true;
            _hasAnyInitialPosition = true;

            gzmsg << "GammaArmControlPlugin: Initial position for joint "
                  << jointName << " = " << value << " rad" << std::endl;
        }
    }

    if (!_hasAnyInitialPosition) {
        gzmsg << "GammaArmControlPlugin: No initial_position_* parameters found. "
              << "Will keep current startup joint positions." << std::endl;
    }
}

void GammaArmControlPlugin::ApplyInitialPositions(
    gz::sim::EntityComponentManager &_ecm)
{
    if (_initialPositionsApplied || !_hasAnyInitialPosition) {
        return;
    }

    for (int i = 0; i < _nJoints; ++i) {
        if (!_hasInitialPosition[i]) {
            continue;
        }

        const double initPos = _initialPositions[i];
        const auto jointEnt = _jointEntities[i];

        auto resetComp =
            _ecm.Component<gz::sim::components::JointPositionReset>(jointEnt);

        if (resetComp) {
            resetComp->Data().clear();
            resetComp->Data().push_back(initPos);
        } else {
            _ecm.CreateComponent(
                jointEnt,
                gz::sim::components::JointPositionReset({initPos}));
        }

        _q(static_cast<unsigned int>(i)) = initPos;
        _qDot(static_cast<unsigned int>(i)) = 0.0;
        _targetPositions[i] = initPos;

        gzmsg << "GammaArmControlPlugin: Applied initial position for joint "
              << _jointNames[i] << " = " << initPos << " rad" << std::endl;
    }

    _initialPositionsApplied = true;
}


bool GammaArmControlPlugin::InitKDL()
{
    // Read URDF from file (path from env var or default)
    const char *urdfFile = std::getenv("GAMMA_URDF_PATH");
    std::string urdfPath;

    if (urdfFile) {
        urdfPath = urdfFile;
    } else {
        gzwarn << "GAMMA_URDF_PATH env not set, trying defaults..." << std::endl;
        const std::vector<std::string> candidates = {
            "/tmp/gamma_arm.urdf",
            "/workspace/VisionFlow-PX4/Tools/simulation/gz/models/gamma_arm/gamma_arm.urdf",
            "/home/renwang/data_storage/VisionFlow-PX4/Tools/simulation/gz/models/gamma_arm/gamma_arm.urdf"
        };
        for (const auto &p : candidates) {
            std::ifstream f(p);
            if (f.good()) {
                urdfPath = p;
                break;
            }
        }
    }

    if (urdfPath.empty()) {
        gzerr << "GammaArmControlPlugin: Cannot find gamma_arm.urdf!" << std::endl;
        return false;
    }

    gzmsg << "GammaArmControlPlugin: Loading URDF from " << urdfPath << std::endl;

    // Read URDF file
    std::ifstream ifs(urdfPath);
    if (!ifs.good()) {
        gzerr << "GammaArmControlPlugin: Cannot read URDF file: " << urdfPath << std::endl;
        return false;
    }
    std::string robotDescStr((std::istreambuf_iterator<char>(ifs)),
                              std::istreambuf_iterator<char>());

    if (robotDescStr.empty()) {
        gzerr << "GammaArmControlPlugin: URDF file is empty!" << std::endl;
        return false;
    }

    try {
        // Remove <visual>...</visual>
        robotDescStr = std::regex_replace(robotDescStr,
            std::regex("<visual[^>]*>.*?</visual>", std::regex::icase), "");
        // Remove <collision>...</collision>
        robotDescStr = std::regex_replace(robotDescStr,
            std::regex("<collision[^>]*>.*?</collision>", std::regex::icase), "");
        // Remove <material>...</material>
        robotDescStr = std::regex_replace(robotDescStr,
            std::regex("<material[^>]*>.*?</material>", std::regex::icase), "");
    } catch (const std::regex_error &e) {
        gzwarn << "GammaArmControlPlugin: regex cleanup failed: " << e.what() << std::endl;
        // Continue with original string - may still work
    }

    // Parse KDL tree
    try {
        KDL::Tree tree;
        if (!kdl_parser::treeFromString(robotDescStr, tree)) {
            gzerr << "GammaArmControlPlugin: Failed to parse KDL tree from URDF!" << std::endl;
            return false;
        }

        // Extract chain from root to tip
        if (!tree.getChain(_kdlRoot, _kdlTip, _chain)) {
            gzerr << "GammaArmControlPlugin: Failed to get KDL chain from '"
                  << _kdlRoot << "' to '" << _kdlTip << "'!" << std::endl;
            return false;
        }

        if (_chain.getNrOfJoints() != static_cast<unsigned int>(_nJoints)) {
            gzwarn << "GammaArmControlPlugin: KDL chain has " << _chain.getNrOfJoints()
                   << " joints, but SDF has " << _nJoints << " joints." << std::endl;
        }

        // Create dynamics solver
        _dynParam.reset(new KDL::ChainDynParam(_chain, _gravity));

        gzmsg << "GammaArmControlPlugin: KDL initialized successfully. "
              << "Chain: " << _kdlRoot << " -> " << _kdlTip
              << " (" << _chain.getNrOfJoints() << " joints)" << std::endl;

        return true;

    } catch (const std::exception &e) {
        gzerr << "GammaArmControlPlugin: KDL init exception: " << e.what() << std::endl;
        return false;
    }
}

void GammaArmControlPlugin::OnJointCmd(int joint_index, const gz::msgs::Double &_msg)
{
    if (joint_index < 0 || joint_index >= _nJoints) {
        return;
    }

    std::lock_guard<std::mutex> lock(_mutex);
    _targetPositions[joint_index] = _msg.data();
}

void GammaArmControlPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
    // Skip if not initialized or no KDL
    if (_nJoints == 0 || !_kdlInitialized) {
        return;
    }

    // Read current joint positions and velocities
    {
        std::lock_guard<std::mutex> lock(_mutex);

        for (int i = 0; i < _nJoints; ++i) {
            auto jointEnt = _jointEntities[i];

            // Get current position
            auto posComp = _ecm.Component<gz::sim::components::JointPosition>(jointEnt);
            if (posComp) {
                _q(static_cast<unsigned int>(i)) = posComp->Data().at(0);
            }

            // Get current velocity
            auto velComp = _ecm.Component<gz::sim::components::JointVelocity>(jointEnt);
            if (velComp) {
                _qDot(static_cast<unsigned int>(i)) = velComp->Data().at(0);
            }

            if (_firstUpdate) {
                if (_hasInitialPosition[i]) {
                    // The actual reset is applied once for all joints below.
                    _targetPositions[i] = _initialPositions[i];
                } else {
                    _targetPositions[i] = _q(static_cast<unsigned int>(i));
                }
            }
        }

        if (_firstUpdate) {
            ApplyInitialPositions(_ecm);
            _firstUpdate = false;
            return; // skip first control cycle after optional reset
        }
    }

    // Compute gravity torques via KDL
    _dynParam->JntToGravity(_q, _gravityTorques);

    // PD control + gravity feedforward
    for (int i = 0; i < _nJoints; ++i) {
        double q_cur = _q(static_cast<unsigned int>(i));
        double qDot_cur = _qDot(static_cast<unsigned int>(i));

        double targetPos = _targetPositions[i];
        double error = targetPos - q_cur;

        // PD: tau = Kp * e - Kd * q_dot (velocity damping, not error derivative)
        double tauPd = _pGains[i] * error - _dGains[i] * qDot_cur;

        // Total torque = PD + gravity compensation
        double tauTotal = tauPd + _gravityTorques(static_cast<unsigned int>(i));

        // Effort saturation
        if (tauTotal > _maxEfforts[i]) {
            tauTotal = _maxEfforts[i];
        } else if (tauTotal < -_maxEfforts[i]) {
            tauTotal = -_maxEfforts[i];
        }

        // Apply force to joint
        auto forceCmd = _ecm.Component<gz::sim::components::JointForceCmd>(_jointEntities[i]);
        if (forceCmd) {
            forceCmd->Data().at(0) = tauTotal;
        } else {
            // Create the component if it doesn't exist
            _ecm.CreateComponent(_jointEntities[i],
                gz::sim::components::JointForceCmd({tauTotal}));
        }
    }
}

} // namespace gamma_arm

// Register the plugin with Gazebo Sim
GZ_ADD_PLUGIN(
    gamma_arm::GammaArmControlPlugin,
    gz::sim::System,
    gamma_arm::GammaArmControlPlugin::ISystemConfigure,
    gamma_arm::GammaArmControlPlugin::ISystemPreUpdate
)
