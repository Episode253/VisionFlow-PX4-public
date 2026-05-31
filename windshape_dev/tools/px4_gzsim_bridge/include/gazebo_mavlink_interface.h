#ifndef GAZEBO_MAVLINK_INTERFACE_HH_
#define GAZEBO_MAVLINK_INTERFACE_HH_

#include <vector>
#include <array>
#include <regex>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <cassert>
#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#include <boost/system/system_error.hpp>

#include <iostream>
#include <random>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include <gz/common5/gz/common.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include "gz/sim/components/Actuators.hh"
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include <gz/math.hh>

#include <common.h>

#include "mavlink_interface.h"
#include "msgbuffer.h"


using lock_guard = std::lock_guard<std::recursive_mutex>;

//! Default distance sensor model joint naming
static const std::regex kDefaultLidarModelLinkNaming("(lidar|sf10a)(.*::link)");
static const std::regex kDefaultSonarModelLinkNaming("(sonar|mb1240-xl-ez4)(.*::link)");

// Default values
static const std::string kDefaultNamespace = "";

// This just proxies the motor commands from command/motor_speed to the single motors via internal
// ConsPtr passing, such that the original commands don't have to go n_motors-times over the wire.
static const std::string kDefaultMotorVelocityReferencePubTopic = "/gazebo/command/motor_speed";

static const std::string kDefaultPoseTopic = "/pose";
static const std::string kDefaultImuTopic = "/imu";
static const std::string kDefaultOpticalFlowTopic = "/px4flow/link/opticalFlow";
static const std::string kDefaultIRLockTopic = "/camera/link/irlock";
static const std::string kDefaultGPSTopic = "/gps";
static const std::string kDefaultVisionTopic = "/vision_odom";
static const std::string kDefaultMagTopic = "/magnetometer";
static const std::string kDefaultBarometerTopic = "/air_pressure";
static const std::string kDefaultCmdVelTopic = "/cmd_vel";

namespace mavlink_interface
{
  class GZ_SIM_VISIBLE GazeboMavlinkInterface:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
  {
    public: GazeboMavlinkInterface();

    public: ~GazeboMavlinkInterface() override;
    public: void Configure(const gz::sim::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &/*_eventMgr*/);
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm);
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
    private:
      gz::common::ConnectionPtr sigIntConnection_;
      std::shared_ptr<MavlinkInterface> mavlink_interface_;
      bool received_first_actuator_{false};
      Eigen::VectorXd motor_input_reference_;
      Eigen::VectorXd servo_input_reference_;
      float cmd_vel_thrust_{0.0};
      float cmd_vel_torque_{0.0};

      gz::sim::Entity entity_{gz::sim::kNullEntity};
      gz::sim::Model model_{gz::sim::kNullEntity};
      gz::sim::Entity modelLink_{gz::sim::kNullEntity};
      std::string model_name_;

      float protocol_version_{2.0};

      double home_latitude_rad_{kDefaultHomeLatitude};
      double home_longitude_rad_{kDefaultHomeLongitude};
      double home_altitude_m_{kDefaultHomeAltitude};
      bool use_serial_{false};

      std::string namespace_{kDefaultNamespace};
      std::string mavlink_control_sub_topic_;
      std::string link_name_;

      bool use_propeller_pid_{false};
      bool use_elevator_pid_{false};
      bool use_left_elevon_pid_{false};
      bool use_right_elevon_pid_{false};

      void PoseCallback(const gz::msgs::Pose_V &_msg);
      void ImuCallback(const gz::msgs::IMU &_msg);
      void BarometerCallback(const gz::msgs::FluidPressure &_msg);
      void MagnetometerCallback(const gz::msgs::Magnetometer &_msg);
      void GpsCallback(const gz::msgs::NavSat &_msg);
      void SendSensorMessages(const gz::sim::UpdateInfo &_info);
      void SendStatusMessages(const gz::sim::UpdateInfo &_info,
          const gz::sim::EntityComponentManager &_ecm);
      void PublishMotorVelocities(gz::sim::EntityComponentManager &_ecm,
          const Eigen::VectorXd &_vels);
      void PublishServoVelocities(const Eigen::VectorXd &_vels);
      void PublishCmdVelocities(const float _thrust, const float _torque);
      void handle_actuator_controls(const gz::sim::UpdateInfo &_info);
      void onSigInt();
      bool IsRunning();
      bool resolveHostName();
      void ResolveWorker();
      float AddSimpleNoise(float value, float mean, float stddev);
      void RotateQuaternion(gz::math::Quaterniond &q_FRD_to_NED,
        const gz::math::Quaterniond q_FLU_to_ENU);
      bool PoseNameMatchesModel(const std::string &pose_name) const;
      uint64_t CurrentWallTimeUsec() const;
      uint64_t HilTimeUsec(const gz::sim::UpdateInfo &_info) const;
      void ParseMulticopterMotorModelPlugins(const std::string &sdfFilePath);

      inline static constexpr unsigned n_out_max = 16;

      double input_offset_[n_out_max];
      std::string joint_control_type_[n_out_max];
      std::string gztopic_[n_out_max];
      double zero_position_disarmed_[n_out_max];
      double zero_position_armed_[n_out_max];
      int motor_input_index_[n_out_max];
      double motor_vel_scalings_[n_out_max] {};
      int servo_input_index_[n_out_max];
      double fallback_motor_velocity_scaling_{1000.0};
      unsigned configured_motor_count_{4};
      bool input_is_cmd_vel_{false};
      bool input_is_cmd_vel_last_{false};

      /// \brief gz communication node and publishers.
      gz::transport::Node node;
      gz::transport::Node::Publisher servo_control_pub_[n_out_max];
      gz::transport::Node::Publisher cmd_vel_pub_;

      std::string pose_sub_topic_{kDefaultPoseTopic};
      std::string imu_sub_topic_{kDefaultImuTopic};
      std::string opticalFlow_sub_topic_{kDefaultOpticalFlowTopic};
      std::string irlock_sub_topic_{kDefaultIRLockTopic};
      std::string gps_sub_topic_{kDefaultGPSTopic};
      std::string vision_sub_topic_{kDefaultVisionTopic};
      std::string mag_sub_topic_{kDefaultMagTopic};
      std::string baro_sub_topic_{kDefaultBarometerTopic};
      std::string cmd_vel_sub_topic_{kDefaultCmdVelTopic};

      std::mutex last_imu_message_mutex_ {};
      std::mutex latest_gps_mutex_ {};

      gz::msgs::IMU last_imu_message_;
      bool has_imu_message_{false};
      bool use_wall_time_for_hil_{true};

      // HITL sensor conditioning. These are intentionally configurable from SDF
      // because PX4 HITL builds and board calibration states can disagree in
      // spectacularly unhelpful ways.
      double hil_accel_scale_{1.0};
      double hil_gyro_scale_{1.0};
      double hil_mag_scale_{1.0};
      bool hil_mag_apply_flu_to_frd_{false};
      bool hil_mag_flip_x_{false};
      bool hil_mag_flip_y_{false};
      bool hil_mag_flip_z_{false};

      // Velocity-estimator safety knobs.
      // In this HITL setup, Gazebo NavSat velocity may produce short spikes at
      // startup / QGC connection time. Keep HIL_GPS velocity disabled by default
      // for static bench HITL, and rely on GPS position + IMU/baro instead.
      bool hil_gps_use_velocity_{false};
      double hil_gps_max_speed_m_s_{5.0};

      // HIL_STATE_QUATERNION is optional here. HIL_SENSOR + HIL_GPS are enough
      // for PX4 EKF bring-up, while an extra ground-truth state stream can fight
      // the estimator if pose timestamps / velocities are not perfectly aligned.
      bool hil_send_state_quaternion_{false};

      bool has_latest_gps_{false};
      double latest_gps_lat_deg_{47.397742};
      double latest_gps_lon_deg_{8.545594};
      double latest_gps_alt_m_{488.0};
      gz::msgs::Actuators motor_velocity_message_;
      unsigned n_motors_detected_{4};
      // Rate limit outgoing HIL messages so QGC parameter traffic and HITL
      // sensor traffic do not starve each other on the same serial link.
      uint64_t last_hil_sensor_send_us_{0};
      uint64_t last_hil_gps_send_us_{0};
      uint64_t last_hil_state_send_us_{0};
      uint64_t hil_sensor_interval_us_{10000};  // 100 Hz
      uint64_t hil_gps_interval_us_{100000};    // 10 Hz
      uint64_t hil_state_interval_us_{50000};   // 20 Hz


      std::chrono::steady_clock::duration last_imu_time_{0};
      std::chrono::steady_clock::duration lastControllerUpdateTime{0};
      std::chrono::steady_clock::duration last_actuator_time_{0};

      bool mag_updated_{false};
      bool baro_updated_;
      bool diff_press_updated_;

      double imu_update_interval_ = 0.004; ///< Used for non-lockstep

      uint64_t status_update_interval_ = 1000; ///< Interval for sending ESC status / info in ms
      uint64_t status_last_update_time_ = 0;

      gz::math::Vector3d gravity_W_{gz::math::Vector3d(0.0, 0.0, -9.8)};
      gz::math::Vector3d velocity_prev_W_;
      gz::math::Vector3d mag_n_;

      double temperature_;
      double pressure_alt_;
      double abs_pressure_;

      bool close_conn_ = false;

      double optflow_distance;
      double sonar_distance;

      bool enable_lockstep_ = false;
      double speed_factor_ = 1.0;
      uint8_t previous_imu_seq_ = 0;
      uint8_t update_skip_factor_ = 1;

      std::string mavlink_hostname_str_;
      struct hostent *hostptr_{nullptr};
      bool mavlink_loaded_{false};
      std::thread hostname_resolver_thread_;

      std::atomic<bool> gotSigInt_ {false};

      std::default_random_engine rnd_gen_;
  };
}

#endif  // GAZEBO_MAVLINK_INTERFACE_HH_
