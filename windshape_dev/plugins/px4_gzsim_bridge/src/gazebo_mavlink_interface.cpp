#include <gazebo_mavlink_interface.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <random>
#include <cmath>
#include <algorithm>
#include <limits>

#include <gz/plugin/Register.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/AirPressureSensor.hh>
#include <gz/sim/components/Magnetometer.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Discovery.hh>
#include <gz/sim/components/Joint.hh>

#define RAD_S_TO_RPM 9.549297

GZ_ADD_PLUGIN(
    mavlink_interface::GazeboMavlinkInterface,
    gz::sim::System,
    mavlink_interface::GazeboMavlinkInterface::ISystemConfigure,
    mavlink_interface::GazeboMavlinkInterface::ISystemPreUpdate,
    mavlink_interface::GazeboMavlinkInterface::ISystemPostUpdate)
using namespace mavlink_interface;

GazeboMavlinkInterface::GazeboMavlinkInterface() :
  motor_input_index_ {},
  servo_input_index_ {}
{
  mavlink_interface_ = std::make_shared<MavlinkInterface>();
  std::fill_n(input_offset_, n_out_max, 0.0);
  std::fill_n(zero_position_disarmed_, n_out_max, 0.0);
  std::fill_n(zero_position_armed_, n_out_max, 0.0);
  std::fill_n(motor_vel_scalings_, n_out_max, fallback_motor_velocity_scaling_);
}

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
  mavlink_interface_->close();
}

void GazeboMavlinkInterface::Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_em) {

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->Get<std::string>("robotNamespace");
  } else {
    gzerr << "[gazebo_mavlink_interface] Please specify a robotNamespace." << std::endl;
  }

  entity_ = _entity;
  model_ = gz::sim::Model(_entity);
  model_name_ = model_.Name(_ecm);

  if (_sdf->HasElement("protocol_version")) {
    protocol_version_ = _sdf->Get<float>("protocol_version");
  }

  gazebo::getSdfParam<std::string>(_sdf, "poseSubTopic", pose_sub_topic_, pose_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "gpsSubTopic", gps_sub_topic_, gps_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "visionSubTopic", vision_sub_topic_, vision_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "opticalFlowSubTopic", opticalFlow_sub_topic_, opticalFlow_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "irlockSubTopic", irlock_sub_topic_, irlock_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_, imu_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "magSubTopic", mag_sub_topic_, mag_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "cmdVelSubTopic", cmd_vel_sub_topic_, cmd_vel_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "baroSubTopic", baro_sub_topic_, baro_sub_topic_);

  double home_latitude_deg = home_latitude_rad_ * 180.0 / M_PI;
  double home_longitude_deg = home_longitude_rad_ * 180.0 / M_PI;
  gazebo::getSdfParam<double>(_sdf, "homeLatitude", home_latitude_deg, home_latitude_deg);
  gazebo::getSdfParam<double>(_sdf, "homeLongitude", home_longitude_deg, home_longitude_deg);
  gazebo::getSdfParam<double>(_sdf, "homeAltitude", home_altitude_m_, home_altitude_m_);
  home_latitude_rad_ = home_latitude_deg * M_PI / 180.0;
  home_longitude_rad_ = home_longitude_deg * M_PI / 180.0;

  if (_sdf->HasElement("use_serial")) {
    use_serial_ = _sdf->Get<bool>("use_serial");
    mavlink_interface_->SetUseSerial(use_serial_);
  }
  if (_sdf->HasElement("serial_device")) {
    mavlink_interface_->SetDevice(_sdf->Get<std::string>("serial_device"));
  }
  if (_sdf->HasElement("serial_baudrate")) {
    mavlink_interface_->SetBaudrate(_sdf->Get<int>("serial_baudrate"));
  }

  // When the bridge owns the serial device, QGC cannot connect to /dev/ttyACM0
  // directly. Enable a small MAVLink UDP forwarder by default in serial mode:
  // PX4 serial -> QGC UDP 14550, and QGC replies -> serial via local UDP 14557.
  bool qgc_udp_forward = use_serial_;
  gazebo::getSdfParam<bool>(_sdf, "qgcUdpForward", qgc_udp_forward, qgc_udp_forward);
  mavlink_interface_->SetQgcUdpForward(qgc_udp_forward);

  std::string qgc_udp_addr = "127.0.0.1";
  int qgc_udp_remote_port = 14550;
  int qgc_udp_local_port = 14557;
  gazebo::getSdfParam<std::string>(_sdf, "qgcUdpAddr", qgc_udp_addr, qgc_udp_addr);
  gazebo::getSdfParam<int>(_sdf, "qgcUdpRemotePort", qgc_udp_remote_port, qgc_udp_remote_port);
  gazebo::getSdfParam<int>(_sdf, "qgcUdpLocalPort", qgc_udp_local_port, qgc_udp_local_port);
  mavlink_interface_->SetQgcUdpAddr(qgc_udp_addr);
  mavlink_interface_->SetQgcUdpRemotePort(qgc_udp_remote_port);
  mavlink_interface_->SetQgcUdpLocalPort(qgc_udp_local_port);

  gzmsg << "[gazebo_mavlink_interface] QGC UDP forwarding is "
        << (qgc_udp_forward ? "enabled" : "disabled")
        << " local_port=" << qgc_udp_local_port
        << " remote=" << qgc_udp_addr << ":" << qgc_udp_remote_port << std::endl;

  double hil_sensor_rate_hz = 100.0;
  double hil_gps_rate_hz = 10.0;
  double hil_state_rate_hz = 20.0;
  gazebo::getSdfParam<double>(_sdf, "hilSensorRateHz", hil_sensor_rate_hz, hil_sensor_rate_hz);
  gazebo::getSdfParam<double>(_sdf, "hilGpsRateHz", hil_gps_rate_hz, hil_gps_rate_hz);
  gazebo::getSdfParam<double>(_sdf, "hilStateRateHz", hil_state_rate_hz, hil_state_rate_hz);
  if (hil_sensor_rate_hz > 1.0) {
    hil_sensor_interval_us_ = static_cast<uint64_t>(1000000.0 / hil_sensor_rate_hz);
  }
  if (hil_gps_rate_hz > 1.0) {
    hil_gps_interval_us_ = static_cast<uint64_t>(1000000.0 / hil_gps_rate_hz);
  }
  if (hil_state_rate_hz > 1.0) {
    hil_state_interval_us_ = static_cast<uint64_t>(1000000.0 / hil_state_rate_hz);
  }
  gazebo::getSdfParam<unsigned>(_sdf, "motorCount", configured_motor_count_, configured_motor_count_);
  configured_motor_count_ = std::max(1u, std::min(configured_motor_count_, n_out_max));
  gazebo::getSdfParam<double>(_sdf, "fallbackMotorVelocityScaling",
                              fallback_motor_velocity_scaling_, fallback_motor_velocity_scaling_);
  fallback_motor_velocity_scaling_ = std::max(1.0, fallback_motor_velocity_scaling_);
  std::fill_n(motor_vel_scalings_, n_out_max, fallback_motor_velocity_scaling_);

  // Real hardware HITL over serial should normally use wall time. Sim time is
  // still available for pure UDP/TCP tests. Mixing both in one link is where
  // estimators go to become modern art.
  gazebo::getSdfParam<bool>(_sdf, "useWallTimeForHil", use_wall_time_for_hil_, use_serial_);

  // Sensor conditioning knobs. Defaults are conservative for generic HITL, but
  // the current q940_ti_0 + CUAV/X7Pro test can set hilAccelScale=2.0 if PX4's
  // simulated accelerometer calibration path reports half gravity.
  gazebo::getSdfParam<double>(_sdf, "hilAccelScale", hil_accel_scale_, hil_accel_scale_);
  gazebo::getSdfParam<double>(_sdf, "hilGyroScale", hil_gyro_scale_, hil_gyro_scale_);
  gazebo::getSdfParam<double>(_sdf, "hilMagScale", hil_mag_scale_, hil_mag_scale_);
  gazebo::getSdfParam<bool>(_sdf, "hilMagApplyFluToFrd", hil_mag_apply_flu_to_frd_, hil_mag_apply_flu_to_frd_);
  gazebo::getSdfParam<bool>(_sdf, "hilMagFlipX", hil_mag_flip_x_, hil_mag_flip_x_);
  gazebo::getSdfParam<bool>(_sdf, "hilMagFlipY", hil_mag_flip_y_, hil_mag_flip_y_);
  gazebo::getSdfParam<bool>(_sdf, "hilMagFlipZ", hil_mag_flip_z_, hil_mag_flip_z_);

  // Safety controls for velocity spikes. For a static HITL bench test, GPS
  // velocity and HIL_STATE_QUATERNION are not required. Disabling them removes
  // two common sources of impossible EKF vertical velocity.
  gazebo::getSdfParam<bool>(_sdf, "hilGpsUseVelocity", hil_gps_use_velocity_, hil_gps_use_velocity_);
  gazebo::getSdfParam<double>(_sdf, "hilGpsMaxSpeed", hil_gps_max_speed_m_s_, hil_gps_max_speed_m_s_);
  gazebo::getSdfParam<bool>(_sdf, "hilSendStateQuaternion", hil_send_state_quaternion_, hil_send_state_quaternion_);

  if (!std::isfinite(hil_accel_scale_) || hil_accel_scale_ <= 0.0) {
    gzerr << "[gazebo_mavlink_interface] Invalid hilAccelScale, forcing 1.0" << std::endl;
    hil_accel_scale_ = 1.0;
  }
  if (!std::isfinite(hil_gyro_scale_) || hil_gyro_scale_ <= 0.0) {
    gzerr << "[gazebo_mavlink_interface] Invalid hilGyroScale, forcing 1.0" << std::endl;
    hil_gyro_scale_ = 1.0;
  }
  if (!std::isfinite(hil_mag_scale_) || hil_mag_scale_ <= 0.0) {
    gzerr << "[gazebo_mavlink_interface] Invalid hilMagScale, forcing 1.0" << std::endl;
    hil_mag_scale_ = 1.0;
  }
  if (!std::isfinite(hil_gps_max_speed_m_s_) || hil_gps_max_speed_m_s_ <= 0.0) {
    gzerr << "[gazebo_mavlink_interface] Invalid hilGpsMaxSpeed, forcing 5.0 m/s" << std::endl;
    hil_gps_max_speed_m_s_ = 5.0;
  }

  gzmsg << "[gazebo_mavlink_interface] HIL sensor conditioning:"
        << " accel_scale=" << hil_accel_scale_
        << " gyro_scale=" << hil_gyro_scale_
        << " mag_scale=" << hil_mag_scale_
        << " mag_apply_flu_to_frd=" << (hil_mag_apply_flu_to_frd_ ? "true" : "false")
        << " mag_flip_xyz=(" << hil_mag_flip_x_ << ","
        << hil_mag_flip_y_ << "," << hil_mag_flip_z_ << ")"
        << " gps_use_velocity=" << (hil_gps_use_velocity_ ? "true" : "false")
        << " gps_max_speed=" << hil_gps_max_speed_m_s_
        << " send_hil_state_quat=" << (hil_send_state_quaternion_ ? "true" : "false")
        << std::endl;

  gzmsg << "[gazebo_mavlink_interface] HIL output rates: sensor=" << hil_sensor_rate_hz
        << "Hz gps=" << hil_gps_rate_hz << "Hz state=" << hil_state_rate_hz
        << "Hz, time_source=" << (use_wall_time_for_hil_ ? "wall" : "sim")
        << ", motor_count=" << configured_motor_count_
        << ", fallback_motor_scaling=" << fallback_motor_velocity_scaling_ << std::endl;

  // Set motor and servo input_reference_ from inputs.control.
  // Important for GzSim MulticopterMotorModel:
  // it expects the model Actuators component to already contain a velocity array
  // whose size covers all motorNumber indexes. If the array is empty at the
  // first physics update, Gazebo prints:
  // "You tried to access index X of the Actuator velocity array which is of size 0".
  // We therefore parse motor plugins and create a zero-filled Actuators component
  // during Configure, before the first PreUpdate.
  motor_input_reference_.resize(n_out_max);
  servo_input_reference_.resize(n_out_max);

  // Parse the MulticopterMotorModel plugins to get the motor velocity scalings
  // and detected motor count. Start with the configured value so a missing SDF
  // parse does not silently create a one-motor aircraft, because apparently
  // chaos needed extra help.
  n_motors_detected_ = configured_motor_count_;
  ParseMulticopterMotorModelPlugins(model_.SourceFilePath(_ecm));

  if (n_motors_detected_ == 0 || n_motors_detected_ > n_out_max) {
    gzerr << "[gazebo_mavlink_interface] Invalid detected motor count: "
          << n_motors_detected_ << ", fallback to 4" << std::endl;
    n_motors_detected_ = 4;
  }

  motor_input_reference_.resize(n_motors_detected_);
  motor_input_reference_.setZero();
  motor_velocity_message_.mutable_velocity()->Resize(n_motors_detected_, 0.0);

  auto existingActuators = _ecm.Component<gz::sim::components::Actuators>(model_.Entity());
  if (existingActuators) {
    existingActuators->SetData(motor_velocity_message_, [](const auto &, const auto &) { return false; });
    _ecm.SetChanged(model_.Entity(), gz::sim::components::Actuators::typeId,
                    gz::sim::ComponentState::PeriodicChange);
  } else {
    _ecm.CreateComponent(model_.Entity(),
                         gz::sim::components::Actuators(motor_velocity_message_));
  }

  gzmsg << "[gazebo_mavlink_interface] initialized Actuators velocity array with "
        << n_motors_detected_ << " motors" << std::endl;

  bool use_tcp = false;
  if (_sdf->HasElement("use_tcp"))
  {
    use_tcp = _sdf->Get<bool>("use_tcp");
    mavlink_interface_->SetUseTcp(use_tcp);
  }

  bool tcp_client_mode = false;
  if (_sdf->HasElement("tcp_client_mode"))
  {
    tcp_client_mode = _sdf->Get<bool>("tcp_client_mode");
    mavlink_interface_->SetUseTcpClientMode(tcp_client_mode);
  }
  gzmsg << "Connecting to PX4 HITL using " << (use_serial_ ? "SERIAL" : (use_tcp ? (tcp_client_mode ? "TCP (client mode)" : "TCP (server mode)") : "UDP")) << std::endl;

  if (_sdf->HasElement("enable_lockstep"))
  {
    enable_lockstep_ = _sdf->Get<bool>("enable_lockstep");
    mavlink_interface_->SetEnableLockstep(enable_lockstep_);
  }
  if (use_serial_ && enable_lockstep_) {
    gzerr << "[gazebo_mavlink_interface] enable_lockstep=true was requested in SERIAL HITL. "
          << "For real flight-controller HITL this is unsafe; forcing lockstep off." << std::endl;
    enable_lockstep_ = false;
    mavlink_interface_->SetEnableLockstep(false);
  }
  gzmsg << "Lockstep is " << (enable_lockstep_ ? "enabled" : "disabled") << std::endl;

  // When running in lockstep, we can run the simulation slower or faster than
  // realtime. The speed can be set using the env variable PX4_SIM_SPEED_FACTOR.
  if (enable_lockstep_)
  {
    const char *speed_factor_str = std::getenv("PX4_SIM_SPEED_FACTOR");
    if (speed_factor_str)
    {
      speed_factor_ = std::atof(speed_factor_str);
      if (!std::isfinite(speed_factor_) || speed_factor_ <= 0.0)
      {
        gzerr << "Invalid speed factor '" << speed_factor_str << "', aborting" << std::endl;
        abort();
      }
    }
    gzmsg << "Speed factor set to: " << speed_factor_ << std::endl;
  }

  // Listen to Ctrl+C / SIGINT.
  sigIntConnection_ = _em.Connect<gz::sim::events::Stop>(std::bind(&GazeboMavlinkInterface::onSigInt, this));

  auto world_name = "/" + gz::sim::scopedName(gz::sim::worldEntity(_ecm), _ecm);

  auto model_name = gz::sim::topicFromScopedName(
    _ecm.EntityByComponents(gz::sim::components::Name(model_name_)), _ecm, false);

  auto vehicle_scope_prefix = world_name + model_name;

  // Publish to servo control
  auto servo_control_topic = model_name + "/servo_";
  for (int i = 0; i < servo_input_reference_.size(); i++) {
    servo_control_pub_[i] = node.Advertise<gz::msgs::Double>(servo_control_topic + std::to_string(i));
  }

  // Publish to cmd vel (for rover control)
  auto cmd_vel_topic = model_name + cmd_vel_sub_topic_;
  cmd_vel_pub_ = node.Advertise<gz::msgs::Twist>(cmd_vel_topic);

  // Subscribe to messages of sensors.
  auto imu_topic = vehicle_scope_prefix + imu_sub_topic_;
  node.Subscribe(imu_topic, &GazeboMavlinkInterface::ImuCallback, this);

  auto baro_topic = vehicle_scope_prefix + baro_sub_topic_;
  node.Subscribe(baro_topic, &GazeboMavlinkInterface::BarometerCallback, this);

  auto mag_topic = vehicle_scope_prefix + mag_sub_topic_;
  node.Subscribe(mag_topic, &GazeboMavlinkInterface::MagnetometerCallback, this);

  auto gps_topic = vehicle_scope_prefix + gps_sub_topic_;
  node.Subscribe(gps_topic, &GazeboMavlinkInterface::GpsCallback, this);

  // Subscribe to entity pose info message
  auto pose_topic = world_name + pose_sub_topic_;
  node.Subscribe(pose_topic, &GazeboMavlinkInterface::PoseCallback, this);

  gzmsg << "[gazebo_mavlink_interface] model: " << model_name_ << std::endl;
  gzmsg << "[gazebo_mavlink_interface] imu topic: " << imu_topic << std::endl;
  gzmsg << "[gazebo_mavlink_interface] gps topic: " << gps_topic << std::endl;
  gzmsg << "[gazebo_mavlink_interface] mag topic: " << mag_topic << std::endl;
  gzmsg << "[gazebo_mavlink_interface] baro topic: " << baro_topic << std::endl;
  gzmsg << "[gazebo_mavlink_interface] pose topic: " << pose_topic << std::endl;
  gzmsg << "[gazebo_mavlink_interface] home: lat=" << home_latitude_deg
        << " lon=" << home_longitude_deg << " alt=" << home_altitude_m_ << std::endl;

  // This doesn't seem to be used anywhere but we leave it here
  // for potential compatibility
  if (_sdf->HasElement("imu_rate")) {
    imu_update_interval_ = 1 / _sdf->Get<int>("imu_rate");
  }

  if (_sdf->HasElement("mavlink_hostname")) {
    mavlink_hostname_str_ = _sdf->Get<std::string>("mavlink_hostname");
    if (! mavlink_hostname_str_.empty()) {
      // Start hostname resolver thread
      hostname_resolver_thread_ = std::thread([this] () {
        ResolveWorker();
      });
    }
  }

  if (_sdf->HasElement("mavlink_addr")) {
    std::string mavlink_addr_str = _sdf->Get<std::string>("mavlink_addr");
    if (mavlink_addr_str != "INADDR_ANY") {
      mavlink_interface_->SetMavlinkAddr(mavlink_addr_str);
    }
  }

  if (_sdf->HasElement("secondary_mavlink_addr")) {
    std::string mavlink_addr_str = _sdf->Get<std::string>("secondary_mavlink_addr");
    if (mavlink_addr_str != "INADDR_ANY") {
      mavlink_interface_->SetSecondaryMavlinkAddr(mavlink_addr_str);
    }
  }

  if (_sdf->HasElement("mavlink_udp_remote_port")) {
    int mavlink_udp_remote_port = _sdf->Get<int>("mavlink_udp_remote_port");
    mavlink_interface_->SetMavlinkUdpRemotePort(mavlink_udp_remote_port);
  }

  if (_sdf->HasElement("mavlink_udp_local_port")) {
    int mavlink_udp_local_port = _sdf->Get<int>("mavlink_udp_local_port");
    mavlink_interface_->SetMavlinkUdpLocalPort(mavlink_udp_local_port);
  }

  if (_sdf->HasElement("secondary_mavlink_udp_local_port")) {
    int mavlink_udp_local_port = _sdf->Get<int>("secondary_mavlink_udp_local_port");
    mavlink_interface_->SetSecondaryMavlinkUdpLocalPort(mavlink_udp_local_port);
  }

  if (_sdf->HasElement("mavlink_tcp_port")) {
    int mavlink_tcp_port = _sdf->Get<int>("mavlink_tcp_port");
    mavlink_interface_->SetMavlinkTcpPort(mavlink_tcp_port);
  }

  mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);

  // set the Mavlink protocol version to use on the link
  if (protocol_version_ == 2.0) {
    chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    gzmsg << "Using MAVLink protocol v2.0" << std::endl;
  }
  else if (protocol_version_ == 1.0) {
    chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    gzmsg << "Using MAVLink protocol v1.0" << std::endl;
  }
  else {
    gzerr << "Unkown protocol version! Using v" << protocol_version_ << "by default " << std::endl;
  }

  std::default_random_engine rnd_gen_;

  if (hostptr_ || mavlink_hostname_str_.empty()) {
    gzmsg << "--> load mavlink_interface_" << std::endl;
    mavlink_interface_->Load();
    mavlink_loaded_ = true;
  }
}

void GazeboMavlinkInterface::PreUpdate(const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm) {

  // Always run at 250 Hz. At 500 Hz, the skip factor should be 2, at 1000 Hz 4.
  if (!(previous_imu_seq_++ % update_skip_factor_ == 0)) {
    return;
  }

  if (!mavlink_loaded_) {
    // mavlink not loaded, exit
    return;
  }

  double dt;

  mavlink_interface_->ReadMAVLinkMessages();

  const uint64_t sim_time_us = HilTimeUsec(_info);

  // Send HIL_SENSOR at a bounded rate. PX4 does not need every Gazebo physics
  // tick here, and sending too much traffic on the same serial link used by QGC
  // causes queue pressure and intermittent QGC disconnects during parameter sync.
  if (last_hil_sensor_send_us_ == 0 || sim_time_us - last_hil_sensor_send_us_ >= hil_sensor_interval_us_) {
    SendSensorMessages(_info);
    last_hil_sensor_send_us_ = sim_time_us;
  }

  handle_actuator_controls(_info);

  if (received_first_actuator_) {
    if (input_is_cmd_vel_ || input_is_cmd_vel_last_) {
      PublishCmdVelocities(cmd_vel_thrust_, cmd_vel_torque_);
      input_is_cmd_vel_last_ = false;
    } else {
      PublishMotorVelocities(_ecm, motor_input_reference_);
      PublishServoVelocities(servo_input_reference_);
    }
  } else {
    // Keep the Actuators component non-empty before PX4 sends the first
    // HIL_ACTUATOR_CONTROLS message. This prevents MulticopterMotorModel from
    // reading an empty velocity array at startup.
    if (motor_input_reference_.size() != n_motors_detected_) {
      motor_input_reference_.resize(n_motors_detected_);
    }
    motor_input_reference_.setZero();
    PublishMotorVelocities(_ecm, motor_input_reference_);
  }
}

void GazeboMavlinkInterface::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) {
  // Send back status data (ESCs) after physics update at a certain interval
  uint64_t current_time = std::chrono::duration_cast<std::chrono::duration<uint64_t>>(_info.simTime * 1e3).count();
  if (current_time - status_last_update_time_ >= status_update_interval_) {
    SendStatusMessages(_info, _ecm);
    status_last_update_time_ = current_time;
  }
}

void GazeboMavlinkInterface::PoseCallback(const gz::msgs::Pose_V &_msg){
  if (!hil_send_state_quaternion_) {
    return;
  }

  for (int p = 0; p < _msg.pose_size(); p++) {
    const std::string pose_name = _msg.pose(p).name();
    if (!PoseNameMatchesModel(pose_name)) {
      continue;
    }

    // Pose_V callbacks are asynchronous and do not carry UpdateInfo, so use wall
    // time here. For real serial HITL this is the desired time base anyway.
    const uint64_t now_usec = CurrentWallTimeUsec();
    if (last_hil_state_send_us_ != 0 && now_usec - last_hil_state_send_us_ < hil_state_interval_us_) {
      return;
    }
    last_hil_state_send_us_ = now_usec;

    const gz::msgs::Vector3d pose_position = _msg.pose(p).position();
    const gz::msgs::Quaternion pose_orientation = _msg.pose(p).orientation();

    // Gazebo pose is ENU + FLU. PX4 HIL_STATE_QUATERNION expects NED + FRD.
    const gz::math::Quaterniond q_flu_to_enu(
      pose_orientation.w(), pose_orientation.x(), pose_orientation.y(), pose_orientation.z());

    gz::math::Quaterniond q_frd_to_ned;
    RotateQuaternion(q_frd_to_ned, q_flu_to_enu);
    q_frd_to_ned.Normalize();

    mavlink_hil_state_quaternion_t hil_state_quat{};
    hil_state_quat.time_usec = now_usec;
    hil_state_quat.attitude_quaternion[0] = q_frd_to_ned.W();
    hil_state_quat.attitude_quaternion[1] = q_frd_to_ned.X();
    hil_state_quat.attitude_quaternion[2] = q_frd_to_ned.Y();
    hil_state_quat.attitude_quaternion[3] = q_frd_to_ned.Z();

    // Convert local ENU position to global GPS coordinates.
    gz::math::Vector3d pos_enu(pose_position.x(), pose_position.y(), pose_position.z());
    double lat_home = home_latitude_rad_;
    double lon_home = home_longitude_rad_;
    double alt_home = home_altitude_m_;
    const auto lat_lon_rad = reproject(pos_enu, lat_home, lon_home, alt_home);

    hil_state_quat.lat = static_cast<int32_t>(lat_lon_rad.first * 180.0 / M_PI * 1e7);
    hil_state_quat.lon = static_cast<int32_t>(lat_lon_rad.second * 180.0 / M_PI * 1e7);
    hil_state_quat.alt = static_cast<int32_t>((home_altitude_m_ + pose_position.z()) * 1000.0);

    // Pose_V does not reliably carry linear/angular velocity for all worlds.
    // Keep these zero for a safe static truth message. The EKF uses HIL_SENSOR/HIL_GPS.
    hil_state_quat.rollspeed = 0.0f;
    hil_state_quat.pitchspeed = 0.0f;
    hil_state_quat.yawspeed = 0.0f;
    hil_state_quat.vx = 0;
    hil_state_quat.vy = 0;
    hil_state_quat.vz = 0;
    hil_state_quat.ind_airspeed = 0;
    hil_state_quat.true_airspeed = 0;
    hil_state_quat.xacc = 0;
    hil_state_quat.yacc = 0;
    hil_state_quat.zacc = -1000;

    mavlink_message_t msg{};
    mavlink_msg_hil_state_quaternion_encode_chan(254, 25, MAVLINK_COMM_0, &msg, &hil_state_quat);
    mavlink_interface_->FinalizeOutgoingMessage(&msg, 254, 25,
      MAVLINK_MSG_ID_HIL_STATE_QUATERNION_MIN_LEN,
      MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN,
      MAVLINK_MSG_ID_HIL_STATE_QUATERNION_CRC);
    mavlink_interface_->PushSendMessage(&msg);
    return;
  }
}

void GazeboMavlinkInterface::ImuCallback(const gz::msgs::IMU &_msg) {
  const std::lock_guard<std::mutex> lock(last_imu_message_mutex_);
  last_imu_message_ = _msg;
  has_imu_message_ = true;
}

void GazeboMavlinkInterface::BarometerCallback(const gz::msgs::FluidPressure &_msg) {
  SensorData::Barometer baro_data;

  const float absolute_pressure = AddSimpleNoise((float) _msg.pressure(), 0, 1.5);
  const float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
  const float pressure_msl = 101325.0f; // pressure at MSL
  const float temperature_msl = 288.0f; // temperature at MSL (Kelvin)

  // Calculate local temperature:
  // absolute_pressure = pressure_msl / pressure_ratio
  // =>
  const float pressure_ratio = pressure_msl / absolute_pressure;
  // pressure_ratio = powf(temperature_msl / temperature_local, 5.256f)
  // =>
  // temperature_local = temperature_msl / powf(pressure_ratio, 1/5.256f)
  const float temperature_local = temperature_msl / powf(pressure_ratio, 0.19025875);

  // Calculate altitude from pressure:
  // temperature_local = temperature_msl - lapse_rate * alt_msl;
  // =>
  const float alt_msl = (temperature_msl - temperature_local) / lapse_rate;

  //gzmsg << "[BarometerCallback] temperature_local: " << temperature_local << " abs_press: " << absolute_pressure << std::endl;

  baro_data.temperature = temperature_local - 273.15f;
  baro_data.abs_pressure = absolute_pressure / 100.0f;
  baro_data.pressure_alt = alt_msl;
  mavlink_interface_->UpdateBarometer(baro_data);
}

void GazeboMavlinkInterface::MagnetometerCallback(const gz::msgs::Magnetometer &_msg) {
  // Gazebo Sim publishes magnetic field in Tesla. In the tested PX4 HITL path
  // the MAVLink receiver / simulator driver expects this value without the old
  // *10000 conversion. The previous conversion made EKF see ~4807 gauss against
  // a ~0.48 gauss reference, which is a delightful way to fail every preflight
  // mag check.
  gz::math::Vector3d mag_body(
    AddSimpleNoise(_msg.field_tesla().x(), 0, 0.0000001),
    AddSimpleNoise(_msg.field_tesla().y(), 0, 0.0000001),
    AddSimpleNoise(_msg.field_tesla().z(), 0, 0.0000001));

  if (hil_mag_apply_flu_to_frd_) {
    mag_body = q_FLU_to_FRD.RotateVector(mag_body);
  }

  mag_body *= hil_mag_scale_;

  if (hil_mag_flip_x_) {
    mag_body.X() *= -1.0;
  }
  if (hil_mag_flip_y_) {
    mag_body.Y() *= -1.0;
  }
  if (hil_mag_flip_z_) {
    mag_body.Z() *= -1.0;
  }

  // Gazebo world frame is ENU: +X=East, +Y=North, +Z=Up.
  // PX4 EKF/QGC yaw is interpreted in NED: +X=North, +Y=East, +Z=Down.
  // Bench test result before this fix:
  //   Gazebo heading East -> PX4/QGC yaw ~= 0 deg (North)
  //   Gazebo heading North -> raw mag horizontal vector ~= [0, -M]
  // This means the horizontal magnetic-field axes are rotated by 90 deg for
  // PX4's FRD expectation. Remap the horizontal components so that:
  //   East  heading -> mag ~= [0, -M, Z]
  //   North heading -> mag ~= [M,  0, Z]
  // Keep Z unchanged because the vertical component was already consistent.
  const gz::math::Vector3d mag_fixed(
      -mag_body.Y(),
      -mag_body.X(),
       mag_body.Z());

  mag_body = mag_fixed;

  if (!std::isfinite(mag_body.X()) ||
      !std::isfinite(mag_body.Y()) ||
      !std::isfinite(mag_body.Z()) ||
      mag_body.Length() < 1e-9) {
    return;
  }

  SensorData::Magnetometer mag_data;
  mag_data.mag_b = Eigen::Vector3d(mag_body.X(), mag_body.Y(), mag_body.Z());
  mavlink_interface_->UpdateMag(mag_data);
}

void GazeboMavlinkInterface::GpsCallback(const gz::msgs::NavSat &_msg) {
  mavlink_hil_gps_t hil_gps_msg{};
  const auto header = _msg.header();
  hil_gps_msg.time_usec = static_cast<uint64_t>((header.stamp().sec() * 1000000) + (header.stamp().nsec() / 1000));
  if (use_wall_time_for_hil_ || hil_gps_msg.time_usec == 0) {
    hil_gps_msg.time_usec = CurrentWallTimeUsec();
  }

  hil_gps_msg.fix_type = 3;
  hil_gps_msg.lat = static_cast<int32_t>(_msg.latitude_deg() * 1e7);
  hil_gps_msg.lon = static_cast<int32_t>(_msg.longitude_deg() * 1e7);
  hil_gps_msg.alt = static_cast<int32_t>(_msg.altitude() * 1000.0);
  hil_gps_msg.eph = 100;
  hil_gps_msg.epv = 120;

  double vn = 0.0;
  double ve = 0.0;
  double vd = 0.0;

  if (hil_gps_use_velocity_) {
    const double raw_vn = _msg.velocity_north();
    const double raw_ve = _msg.velocity_east();
    const double raw_vd = -_msg.velocity_up();
    const Eigen::Vector3d raw_v(raw_vn, raw_ve, raw_vd);

    if (std::isfinite(raw_vn) && std::isfinite(raw_ve) && std::isfinite(raw_vd) &&
        raw_v.norm() <= hil_gps_max_speed_m_s_) {
      vn = raw_vn;
      ve = raw_ve;
      vd = raw_vd;
    } else {
      static uint64_t rejected_gps_velocity_count = 0;
      if (rejected_gps_velocity_count++ < 20 || rejected_gps_velocity_count % 100 == 0) {
        gzerr << "[gazebo_mavlink_interface] rejected implausible GPS velocity NED=("
              << raw_vn << "," << raw_ve << "," << raw_vd << ") m/s, norm="
              << raw_v.norm() << " > max=" << hil_gps_max_speed_m_s_
              << "; sending zero GPS velocity" << std::endl;
      }
    }
  }

  Eigen::Vector3d v(vn, ve, vd);
  hil_gps_msg.vel = static_cast<uint16_t>(std::min(65535.0, v.norm() * 100.0));
  hil_gps_msg.vn = static_cast<int16_t>(gazebo::constrain(vn * 100.0, -32768.0, 32767.0));
  hil_gps_msg.ve = static_cast<int16_t>(gazebo::constrain(ve * 100.0, -32768.0, 32767.0));
  hil_gps_msg.vd = static_cast<int16_t>(gazebo::constrain(vd * 100.0, -32768.0, 32767.0));

  if (v.norm() > 0.05) {
    gz::math::Angle cog(atan2(ve, vn));
    cog.Normalize();
    hil_gps_msg.cog = static_cast<uint16_t>(gazebo::GetDegrees360(cog) * 100.0);
  } else {
    hil_gps_msg.cog = 0;
  }
  hil_gps_msg.satellites_visible = 12;
  hil_gps_msg.id = 0;

  {
    std::lock_guard<std::mutex> lock(latest_gps_mutex_);
    has_latest_gps_ = true;
    latest_gps_lat_deg_ = _msg.latitude_deg();
    latest_gps_lon_deg_ = _msg.longitude_deg();
    latest_gps_alt_m_ = _msg.altitude();
  }

  const uint64_t now_usec = CurrentWallTimeUsec();
  if (last_hil_gps_send_us_ != 0 && now_usec - last_hil_gps_send_us_ < hil_gps_interval_us_) {
    return;
  }
  last_hil_gps_send_us_ = now_usec;

  mavlink_message_t msg{};
  mavlink_msg_hil_gps_encode_chan(254, 25, MAVLINK_COMM_0, &msg, &hil_gps_msg);
  mavlink_interface_->FinalizeOutgoingMessage(&msg, 254, 25,
    MAVLINK_MSG_ID_HIL_GPS_MIN_LEN,
    MAVLINK_MSG_ID_HIL_GPS_LEN,
    MAVLINK_MSG_ID_HIL_GPS_CRC);
  mavlink_interface_->PushSendMessage(&msg);
}

void GazeboMavlinkInterface::SendSensorMessages(const gz::sim::UpdateInfo &_info) {
  gz::msgs::IMU last_imu_message;
  bool has_imu = false;
  {
    const std::lock_guard<std::mutex> lock(last_imu_message_mutex_);
    last_imu_message = last_imu_message_;
    has_imu = has_imu_message_;
  }

  if (has_imu) {
    const gz::math::Vector3d accel_flu(
      last_imu_message.linear_acceleration().x(),
      last_imu_message.linear_acceleration().y(),
      last_imu_message.linear_acceleration().z());

    const gz::math::Vector3d gyro_flu(
      last_imu_message.angular_velocity().x(),
      last_imu_message.angular_velocity().y(),
      last_imu_message.angular_velocity().z());

    const gz::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector(accel_flu) * hil_accel_scale_;
    const gz::math::Vector3d gyro_b = q_FLU_to_FRD.RotateVector(gyro_flu) * hil_gyro_scale_;

    if (std::isfinite(accel_b.X()) && std::isfinite(accel_b.Y()) && std::isfinite(accel_b.Z()) &&
        std::isfinite(gyro_b.X()) && std::isfinite(gyro_b.Y()) && std::isfinite(gyro_b.Z())) {
      SensorData::Imu imu_data;
      imu_data.accel_b = Eigen::Vector3d(accel_b.X(), accel_b.Y(), accel_b.Z());
      imu_data.gyro_b = Eigen::Vector3d(gyro_b.X(), gyro_b.Y(), gyro_b.Z());
      mavlink_interface_->UpdateIMU(imu_data);
    }
  }

  // If no IMU has arrived yet, do not overwrite MavlinkInterface's safe
  // constructor fallback. Empty IMU packets are an excellent way to teach PX4
  // nihilism, so we avoid that.
  mavlink_interface_->SendSensorMessages(HilTimeUsec(_info));
}

void GazeboMavlinkInterface::SendStatusMessages(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) {
  const uint64_t time_usec = HilTimeUsec(_info);
  struct StatusData::EscStatus status{};

  for (int i = 0; i < static_cast<int>(std::min<unsigned>(n_motors_detected_, MAX_N_ESCS)); ++i) {
    const std::string joint_name = "rotor_" + std::to_string(i) + "_joint";
    const gz::sim::Entity joint_entity = _ecm.EntityByComponents(
      gz::sim::components::Name(joint_name), gz::sim::components::Joint());

    if (joint_entity == gz::sim::kNullEntity) {
      continue;
    }

    const auto joint_velocity = _ecm.ComponentData<gz::sim::components::JointVelocity>(joint_entity);
    if (joint_velocity && !joint_velocity->empty()) {
      status.esc[i].rpm = static_cast<int>((*joint_velocity)[0] * RAD_S_TO_RPM);
      status.esc_count = std::max(status.esc_count, i + 1);
    }
  }

  if (status.esc_count > 0) {
    mavlink_interface_->SendEscStatusMessages(time_usec, status);
  }
}

void GazeboMavlinkInterface::handle_actuator_controls(const gz::sim::UpdateInfo &_info) {
  bool armed = mavlink_interface_->GetArmedState();

  last_actuator_time_ = _info.simTime;

  Eigen::VectorXd actuator_controls = mavlink_interface_->GetActuatorControls();
  if (actuator_controls.size() < n_out_max) return; //TODO: Handle this properly

  // Read Cmd vel input for rover
  if (actuator_controls[n_out_max - 1] != 0.0 || actuator_controls[n_out_max - 2] != 0.0) {
    cmd_vel_thrust_ = actuator_controls[n_out_max - 1];
    cmd_vel_torque_ = actuator_controls[n_out_max - 2];
    input_is_cmd_vel_ = true;
    received_first_actuator_ = mavlink_interface_->GetReceivedFirstActuator();
    return;
  } else {
    // Send last cmd_vel once to zero it out
    if (input_is_cmd_vel_) {
      cmd_vel_thrust_ = 0.0;
      cmd_vel_torque_ = 0.0;
      input_is_cmd_vel_last_ = true;
    }
    input_is_cmd_vel_ = false;
  }

  // Read Input References for servos
  if (servo_input_reference_.size() == n_out_max) {
    unsigned n_servos = 0;
    for (unsigned i = 0; i < n_out_max; i++) {
      if (!mavlink_interface_->IsInputMotorAtIndex(i)) {
        servo_input_index_[n_servos++] = i;
      }
    }
    servo_input_reference_.resize(n_servos);
  }

  for (int i = 0; i < servo_input_reference_.size(); i++) {
    if (armed) {
      servo_input_reference_[i] = actuator_controls[servo_input_index_[i]];
    } else {
      servo_input_reference_[i] = 0;
    }
  }

  // Read Input References for motors.
  // Keep output vector size fixed to detected motor count. Do not shrink it to
  // zero when PX4 disarmed flags are not available yet, otherwise Gazebo motor
  // plugins will keep seeing an empty Actuators.velocity array.
  if (motor_input_reference_.size() != n_motors_detected_) {
    motor_input_reference_.resize(n_motors_detected_);
  }

  for (unsigned i = 0; i < n_motors_detected_; i++) {
    // PX4 HIL_ACTUATOR_CONTROLS motor outputs are normally in controls[0..N-1]
    // for quadrotor HITL. If flags are valid, still allow the direct mapping.
    motor_input_index_[i] = i;

    if (armed) {
      const double u = gazebo::constrain(actuator_controls[motor_input_index_[i]], 0.0, 1.0);
      motor_input_reference_[i] = u * motor_vel_scalings_[i];
    } else {
      motor_input_reference_[i] = 0.0;
    }
  }

  received_first_actuator_ = mavlink_interface_->GetReceivedFirstActuator();
}

bool GazeboMavlinkInterface::IsRunning()
{
  return true; //TODO;
}

void GazeboMavlinkInterface::onSigInt() {
  mavlink_interface_->onSigInt();
}

// The following snippet was copied from https://github.com/gzrobotics/ign-gazebo/blob/ign-gazebo4/src/systems/multicopter_control/MulticopterVelocityControl.cc
void GazeboMavlinkInterface::PublishMotorVelocities(
    gz::sim::EntityComponentManager &_ecm,
    const Eigen::VectorXd &_vels)
{
  if (_vels.size() != motor_velocity_message_.velocity_size())
  {
    motor_velocity_message_.mutable_velocity()->Resize(_vels.size(), 0);
  }
  for (int i = 0; i < _vels.size(); ++i)
  {
    motor_velocity_message_.set_velocity(i, _vels(i));
  }
  // Publish the message by setting the Actuators component on the model entity.
  // This assumes that the MulticopterMotorModel system is attached to this
  // model
  auto actuatorMsgComp =
      _ecm.Component<gz::sim::components::Actuators>(model_.Entity());

  if (actuatorMsgComp)
  {
    auto compFunc = [](const gz::msgs::Actuators &_a, const gz::msgs::Actuators &_b)
    {
      if (_a.velocity_size() != _b.velocity_size()) {
        return false;
      }
      return std::equal(_a.velocity().begin(), _a.velocity().end(),
                        _b.velocity().begin());
    };
    auto state = actuatorMsgComp->SetData(this->motor_velocity_message_, compFunc)
                     ? gz::sim::ComponentState::PeriodicChange
                     : gz::sim::ComponentState::NoChange;
    _ecm.SetChanged(model_.Entity(), gz::sim::components::Actuators::typeId, state);
  }
  else
  {
    _ecm.CreateComponent(model_.Entity(),
                         gz::sim::components::Actuators(this->motor_velocity_message_));
  }
}

void GazeboMavlinkInterface::PublishServoVelocities(const Eigen::VectorXd &_vels)
{
  for (int i = 0; i < _vels.size(); i++) {
    gz::msgs::Double servo_input;
    servo_input.set_data(_vels(i));
    servo_control_pub_[i].Publish(servo_input);
  }
}

void GazeboMavlinkInterface::PublishCmdVelocities(const float _thrust, const float _torque)
{
  gz::msgs::Twist cmd_vel_message;
  cmd_vel_message.mutable_linear()->set_x(_thrust);
  cmd_vel_message.mutable_angular()->set_z(_torque);

  if (cmd_vel_pub_.Valid()) {
    cmd_vel_pub_.Publish(cmd_vel_message);
  }
}

bool GazeboMavlinkInterface::resolveHostName()
{
  if (!mavlink_hostname_str_.empty()) {
    gzmsg << "Try to resolve hostname: '"  << mavlink_hostname_str_ << "'" << std::endl;
    hostptr_ = gethostbyname(mavlink_hostname_str_.c_str());
    if (hostptr_ && hostptr_->h_length && hostptr_->h_addrtype == AF_INET) {
      struct in_addr **addr_l = (struct in_addr **)hostptr_->h_addr_list;
      char *addr_str = inet_ntoa(*addr_l[0]);
      std::string ip_addr = std::string(addr_str);
      mavlink_interface_->SetMavlinkAddr(ip_addr);
      gzmsg << "Host name '" << mavlink_hostname_str_ << "' resolved to IP: " << ip_addr << std::endl;
      return true;
    }
    return false;
  } else {
    // Assume resolved in case hostname is not given at all
    return true;
  }

}

void GazeboMavlinkInterface::ResolveWorker()
{
  gzmsg << "[ResolveWorker] Start Resolving hostname" << std::endl;
  while (!resolveHostName()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  gzmsg << "[ResolveWorker] --> load mavlink_interface_" << std::endl;
  mavlink_interface_->Load();
  mavlink_loaded_ = true;
}

float GazeboMavlinkInterface::AddSimpleNoise(float value, float mean, float stddev) {
  std::normal_distribution<float> dist(mean, stddev);
  return value + dist(rnd_gen_);
}


bool GazeboMavlinkInterface::PoseNameMatchesModel(const std::string &pose_name) const
{
  if (pose_name == model_name_) {
    return true;
  }
  const std::string scoped_suffix = "::" + model_name_;
  return pose_name.size() >= scoped_suffix.size() &&
         pose_name.compare(pose_name.size() - scoped_suffix.size(), scoped_suffix.size(), scoped_suffix) == 0;
}

uint64_t GazeboMavlinkInterface::CurrentWallTimeUsec() const
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
}

uint64_t GazeboMavlinkInterface::HilTimeUsec(const gz::sim::UpdateInfo &_info) const
{
  if (use_wall_time_for_hil_) {
    return CurrentWallTimeUsec();
  }
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(_info.simTime).count());
}

void GazeboMavlinkInterface::RotateQuaternion(gz::math::Quaterniond &q_FRD_to_NED,
    const gz::math::Quaterniond q_FLU_to_ENU)
{
	// FLU (ROS) to FRD (PX4) static rotation
	static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

	/**
	 * @brief Quaternion for rotation between ENU and NED frames
	 *
	 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
	 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
	 * This rotation is symmetric, so q_ENU_to_NED == q_NED_to_ENU.
	 */
	static const auto q_ENU_to_NED = gz::math::Quaterniond(0, 0.70711, 0.70711, 0);

	// final rotation composition
	q_FRD_to_NED = q_ENU_to_NED * q_FLU_to_ENU * q_FLU_to_FRD.Inverse();
}

void GazeboMavlinkInterface::ParseMulticopterMotorModelPlugins(const std::string &sdfFilePath)
{
  if (sdfFilePath.empty()) {
    gzerr << "[gazebo_mavlink_interface] Empty model SourceFilePath; using configured motor count "
          << n_motors_detected_ << " and fallback scaling "
          << fallback_motor_velocity_scaling_ << std::endl;
    return;
  }

  sdf::Root root;
  const sdf::Errors errors = root.Load(sdfFilePath);
  if (!errors.empty())
  {
    for (const auto &error : errors)
    {
      gzerr << "[gazebo_mavlink_interface] SDF parse error: " << error.Message() << std::endl;
    }
    gzerr << "[gazebo_mavlink_interface] Using configured motor count " << n_motors_detected_
          << " with fallback scaling " << fallback_motor_velocity_scaling_ << std::endl;
    return;
  }

  const sdf::Model *model = root.Model();
  if (!model)
  {
    gzerr << "[gazebo_mavlink_interface] No root model found in SDF file; using configured motor count."
          << std::endl;
    return;
  }

  unsigned detected_count = 0;
  for (const sdf::Plugin &plugin : model->Plugins())
  {
    const std::string plugin_name = plugin.Name();
    const std::string plugin_file = plugin.Filename();
    if (plugin_name.find("MulticopterMotorModel") == std::string::npos &&
        plugin_file.find("multicopter-motor-model") == std::string::npos) {
      continue;
    }

    if (!plugin.Element() || !plugin.Element()->HasElement("motorNumber")) {
      continue;
    }

    const int motorNumber = plugin.Element()->Get<int>("motorNumber");
    if (motorNumber < 0 || motorNumber >= static_cast<int>(n_out_max))
    {
      gzerr << "[gazebo_mavlink_interface] Ignoring invalid motorNumber=" << motorNumber
            << ", allowed range is [0," << (n_out_max - 1) << "]" << std::endl;
      continue;
    }

    detected_count = std::max<unsigned>(detected_count, static_cast<unsigned>(motorNumber + 1));

    if (plugin.Element()->HasElement("maxRotVelocity"))
    {
      const double scale = plugin.Element()->Get<double>("maxRotVelocity");
      if (std::isfinite(scale) && scale > 0.0) {
        motor_vel_scalings_[motorNumber] = scale;
      }
    }
  }

  if (detected_count > 0) {
    n_motors_detected_ = std::max(n_motors_detected_, detected_count);
  }

  gzmsg << "[gazebo_mavlink_interface] motor scaling:";
  for (unsigned i = 0; i < n_motors_detected_; ++i) {
    gzmsg << " motor" << i << "=" << motor_vel_scalings_[i];
  }
  gzmsg << std::endl;
}
