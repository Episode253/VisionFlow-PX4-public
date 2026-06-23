/****************************************************************************
 *@UserPositionControl.hpp
 *Author: Huazi Cao
 *Derived from module mc_att_control
 ****************************************************************************/

#include "usr_att_control.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/mathlib.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <matrix/matrix/math.hpp>
#include <math.h>



using namespace matrix;

UserAttitudeControl::UserAttitudeControl(bool vtol) :
        ModuleParams(nullptr),
        WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
        _vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
        _actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0)),
        _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
        _vtol(vtol)
{
    _vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
    if (_vtol) {
            int32_t vt_type = -1;

            if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
                    _vtol_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
            }
    }

    parameters_updated();
}

UserAttitudeControl::~UserAttitudeControl()
{
    perf_free(_loop_perf);
}

bool UserAttitudeControl::init()
{
    if (!_vehicle_attitude_sub.registerCallback()) {
       PX4_ERR("vehicle_attitude callback registration failed!");
        return false;
    }
    //ScheduleOnInterval(2000_us);
    return true;
}

void UserAttitudeControl::parameters_updated()
{

    _attitude_control.setControllerGain(Vector3f(_param_usr_lambda_q_x.get(), _param_usr_lambda_q_y.get(), _param_usr_lambda_q_z.get()),
                                  Vector3f(_param_usr_k_q_x.get(), _param_usr_k_q_y.get(), _param_usr_k_q_z.get()));


 _attitude_control.setCESOParas(
        Vector3f(_param_usr_eso_l_x.get(), _param_usr_eso_l_y.get(), _param_usr_eso_l_z.get()),
        _param_usr_eso_epsi.get(),_param_usr_eso_c1.get(),_param_usr_eso_c2.get());

 _attitude_control.setPresetTrajParas(_param_PresetTraj_l.get(), _param_PresetTraj_w.get(), _param_PresetTraj_epsilon.get(), _param_PresetTraj_k.get());


    // set Inertia Matrix of the UAV
     _mass_uav = _param_usr_mass_uav.get();
     _mass_manip =  _param_usr_mass_manip.get();
    float inertia_b[9] ={ _param_usr_i_xx.get(),-_param_usr_i_xy.get(),-_param_usr_i_xz.get(),
                         -_param_usr_i_xy.get(), _param_usr_i_yy.get(),-_param_usr_i_yz.get(),
                         -_param_usr_i_xz.get(),-_param_usr_i_yz.get(), _param_usr_i_zz.get()};
    SquareMatrix<float, 3> inertia_matrix(inertia_b);
    _attitude_control.setInertiaMatrix(inertia_matrix);
    // angular rate limits
    using math::radians;
    _attitude_control.setRateLimit(Vector3f(radians(_param_usr_rollrate_max.get()), radians(_param_usr_pitchrate_max.get()),
                                            radians(_param_usr_yawrate_max.get())));

    _man_tilt_max = math::radians(_param_usr_man_tilt_max.get());
 _actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);
}

float UserAttitudeControl::throttle_curve(float throttle_stick_input)
{
        const float throttle_min = _landed ? 0.0f : _param_usr_manthr_min.get();

        // throttle_stick_input is in range [0, 1]
        switch (_param_usr_thr_curve.get()) {
        case 1: // no rescaling to hover throttle
                return throttle_min + throttle_stick_input * (_param_usr_thr_max.get() - throttle_min);

        default: // 0 or other: rescale to hover throttle at 0.5 stick
                return math::gradual3(throttle_stick_input,
                                      0.f, .5f, 1.f,
                                      throttle_min, _param_usr_thr_hover.get(), _param_usr_thr_max.get());
        }
}

void UserAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt, bool reset_yaw_sp)
{
        vehicle_attitude_setpoint_s attitude_setpoint{};
        const float yaw = Eulerf(q).psi();

        /* reset yaw setpoint to current position if needed */
        if (reset_yaw_sp) {
                _man_yaw_sp = yaw;

        } else if (math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f) > 0.05f
                   || _param_usr_airmode.get() == (int32_t)Mixer::Airmode::roll_pitch_yaw) {

                const float yaw_rate = math::radians(_param_usr_man_y_max.get());
                attitude_setpoint.yaw_sp_move_rate = _manual_control_setpoint.r * yaw_rate;
                _man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
        }

        /*
         * Input mapping for roll & pitch setpoints
         * ----------------------------------------
         * We control the following 2 angles:
         * - tilt angle, given by sqrt(x*x + y*y)
         * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
         *
         * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
         * points to, and changes of the stick input are linear.
         */
        _man_x_input_filter.setParameters(dt, _param_usr_man_tilt_tau.get());
        _man_y_input_filter.setParameters(dt, _param_usr_man_tilt_tau.get());
        _man_x_input_filter.update(_manual_control_setpoint.x * _man_tilt_max);
        _man_y_input_filter.update(_manual_control_setpoint.y * _man_tilt_max);
        const float x = _man_x_input_filter.getState();
        const float y = _man_y_input_filter.getState();

        // we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
        Vector2f v = Vector2f(y, -x);
        float v_norm = v.norm(); // the norm of v defines the tilt angle

        if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
                v *= _man_tilt_max / v_norm;
        }

        Quatf q_sp_rpy = AxisAnglef(v(0), v(1), 0.f);
        Eulerf euler_sp = q_sp_rpy;
        attitude_setpoint.roll_body = euler_sp(0);
        attitude_setpoint.pitch_body = euler_sp(1);
        // The axis angle can change the yaw as well (noticeable at higher tilt angles).
        // This is the formula by how much the yaw changes:
        //   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
        //   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
        attitude_setpoint.yaw_body = _man_yaw_sp + euler_sp(2);

        /* modify roll/pitch only if we're a VTOL */
        if (_vtol) {
                // Construct attitude setpoint rotation matrix. Modify the setpoints for roll
                // and pitch such that they reflect the user's intention even if a large yaw error
                // (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
                // from the pure euler angle setpoints will lead to unexpected attitude behaviour from
                // the user's view as the euler angle sequence uses the  yaw setpoint and not the current
                // heading of the vehicle.
                // However there's also a coupling effect that causes oscillations for fast roll/pitch changes
                // at higher tilt angles, so we want to avoid using this on multicopters.
                // The effect of that can be seen with:
                // - roll/pitch into one direction, keep it fixed (at high angle)
                // - apply a fast yaw rotation
                // - look at the roll and pitch angles: they should stay pretty much the same as when not yawing

                // calculate our current yaw error
                float yaw_error = wrap_pi(attitude_setpoint.yaw_body - yaw);

                // compute the vector obtained by rotating a z unit vector by the rotation
                // given by the roll and pitch commands of the user
                Vector3f zB = {0.0f, 0.0f, 1.0f};
                Dcmf R_sp_roll_pitch = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, 0.0f);
                Vector3f z_roll_pitch_sp = R_sp_roll_pitch * zB;

                // transform the vector into a new frame which is rotated around the z axis
                // by the current yaw error. this vector defines the desired tilt when we look
                // into the direction of the desired heading
                Dcmf R_yaw_correction = Eulerf(0.0f, 0.0f, -yaw_error);
                z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

                // use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
                // R_tilt is computed from_euler; only true if cos(roll) not equal zero
                // -> valid if roll is not +-pi/2;
                attitude_setpoint.roll_body = -asinf(z_roll_pitch_sp(1));
                attitude_setpoint.pitch_body = atan2f(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
        }

        /* copy quaternion setpoint to attitude setpoint topic */
        Quatf q_sp = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body);
        q_sp.copyTo(attitude_setpoint.q_d);

        attitude_setpoint.thrust_body[2] = -throttle_curve(math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f));
        attitude_setpoint.timestamp = hrt_absolute_time();

        _vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}


void UserAttitudeControl::Run()
{
    if (should_exit()) {
        _vehicle_attitude_sub.unregisterCallback();
        exit_and_cleanup();
        return;
    }
    perf_begin(_loop_perf);

    // DO WORK
    // Check if parameters have changed
    if (_parameter_update_sub.updated()) {
        // clear update
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        updateParams();
        parameters_updated();
    }

    	vehicle_local_position_s local_pos; //obtain the vehicle position
        if (_local_pos_sub.update(&local_pos)) {
                pos_z = local_pos.z;

        }


    // run controller on attitude updates
    vehicle_attitude_s v_att;
    /* run controller on gyro changes */
    vehicle_angular_velocity_s angular_velocity;
    if (_vehicle_angular_velocity_sub.update(&angular_velocity))
    {
        _vehicle_attitude_sub.update(&v_att);
        // Check for new attitude setpoint
        if (_vehicle_attitude_setpoint_sub.updated()) {
                vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
                _vehicle_attitude_setpoint_sub.update(&vehicle_attitude_setpoint);
                _attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
                _thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
        }
        // Check for a heading reset
        if (_quat_reset_counter != v_att.quat_reset_counter) {
                const Quatf delta_q_reset(v_att.delta_q_reset);
                // for stabilized attitude generation only extract the heading change from the delta quaternion
                _man_yaw_sp += Eulerf(delta_q_reset).psi();
                _attitude_control.adaptAttitudeSetpoint(delta_q_reset);

                _quat_reset_counter = v_att.quat_reset_counter;
        }
        // grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
        vehicle_angular_acceleration_s v_angular_acceleration{};
        _vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

        // Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
        const float dt = math::constrain(((angular_velocity.timestamp - _last_run) * 1e-6f), 0.0002f, 0.02f);
        _last_run = angular_velocity.timestamp;
        // double  tt = dt;
        // PX4_WARN("_dt = %f\n",tt);


        const Vector3f angular_accel{v_angular_acceleration.xyz};
        const Vector3f rates{angular_velocity.xyz};


        /* check for updates in other topics */
        _manual_control_setpoint_sub.update(&_manual_control_setpoint);

        _v_control_mode_sub.update(&_v_control_mode);

        if (_vehicle_land_detected_sub.updated()) {
                vehicle_land_detected_s vehicle_land_detected;

                if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
                        _landed = vehicle_land_detected.landed;
                        _maybe_landed = vehicle_land_detected.maybe_landed;
                }
        }

        if (_vehicle_status_sub.updated()) {
                vehicle_status_s vehicle_status;

                if (_vehicle_status_sub.copy(&vehicle_status)) {
                        _vehicle_type_rotary_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
                        _vtol = vehicle_status.is_vtol;
                        _vtol_in_transition_mode = vehicle_status.in_transition_mode;
                }
        }

        if (_landing_gear_sub.updated()) {
                landing_gear_s landing_gear;

                if (_landing_gear_sub.copy(&landing_gear)) {
                        if (landing_gear.landing_gear != landing_gear_s::GEAR_KEEP) {
                                _landing_gear = landing_gear.landing_gear;
                        }
                }
        }

        bool attitude_setpoint_generated = false;

        const bool is_hovering = (_vehicle_type_rotary_wing && !_vtol_in_transition_mode);

        // vehicle is a tailsitter in transition mode
        const bool is_tailsitter_transition = (_vtol_tailsitter && _vtol_in_transition_mode);
        // attude control is running when flag_control_attitude_enabled is true and vehicle is a tailsitter in transition mode
        bool run_att_ctrl = _v_control_mode.flag_control_attitude_enabled && (is_hovering || is_tailsitter_transition);
//        PX4_WARN("run_att_ctrl = %d\n",run_att_ctrl);

        // Reset ESO
        if (_v_control_mode.flag_control_rates_enabled && !_actuators_0_circuit_breaker_enabled) {
            // reset integral if disarmed
            if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
                _attitude_control.resetESO();
                _attitude_control.resetPresetTraj();
            }
        }
        // to ban flag_control_manual_enabled
//        bool manual_has_been_changed_to_att = _v_control_mode.flag_control_manual_enabled && !_v_control_mode.flag_control_attitude_enabled;
//        if (run_att_ctrl || manual_has_been_changed_to_att) {
        if (run_att_ctrl) {
            const Quatf q{v_att.q};

            // update saturation status from mixer feedback
            if (_motor_limits_sub.updated()) {
                multirotor_motor_limits_s motor_limits;

                if (_motor_limits_sub.copy(&motor_limits)) {
                }
            }

            // Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
            if (_v_control_mode.flag_control_manual_enabled &&
                !_v_control_mode.flag_control_altitude_enabled &&
                !_v_control_mode.flag_control_velocity_enabled &&
                !_v_control_mode.flag_control_position_enabled) {

                    generate_attitude_setpoint(q, dt, _reset_yaw_sp);
                    attitude_setpoint_generated = true;

            } else {
                    _man_x_input_filter.reset(0.f);
                    _man_y_input_filter.reset(0.f);
            }


            Vector3f rates_sp;
           _attitude_control.update(q,rates, dt, _landed, _torque,rates_sp,pos_z);

           // publish rate setpoint
           vehicle_rates_setpoint_s v_rates_sp{};
           v_rates_sp.roll = rates_sp(0);
           v_rates_sp.pitch = rates_sp(1);
           v_rates_sp.yaw = rates_sp(2);
           _thrust_setpoint_body.copyTo(v_rates_sp.thrust_body);
           v_rates_sp.timestamp = hrt_absolute_time();

           _v_rates_sp_pub.publish(v_rates_sp);
            // handle thrust set point
            _thrust_sp = - _thrust_setpoint_body(2);
            _torque =_torque*(1.0f/_param_usr_tau_coe.get());
            _torque(0)= math::constrain(_torque(0),-1.0f,1.0f);
            _torque(1)= math::constrain(_torque(1),-1.0f,1.0f);
            _torque(2)= math::constrain(_torque(2),-1.0f,1.0f);
//            _thrust_sp= math::constrain(_thrust_sp, 0.0f, 1.0f);

            // publish rate controller status
            rate_ctrl_status_s rate_ctrl_status{};
            _attitude_control.getRateControlStatus(rate_ctrl_status);
            rate_ctrl_status.timestamp = hrt_absolute_time();
            _controller_status_pub.publish(rate_ctrl_status);
//            PX4_WARN("test is running !\n");
            // publish actuator controls
            actuator_controls_s actuators{};
            actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(_torque(0)) ? _torque(0) : 0.0f;
            actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(_torque(1)) ? _torque(1) : 0.0f;
            actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(_torque(2)) ? _torque(2) : 0.0f;
            actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
            actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = _landing_gear;
            actuators.timestamp_sample = v_att.timestamp;

            // scale effort by battery status if enabled
            if (_param_usr_bat_scale_en.get()) {
                    if (_battery_status_sub.updated()) {
                            battery_status_s battery_status;

                            if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
                                    _battery_status_scale = battery_status.scale;
                            }
                    }

                    if (_battery_status_scale > 0.0f) {
                            for (int i = 0; i < 4; i++) {
                                    actuators.control[i] *= _battery_status_scale;
                            }
                    }
            }

            actuators.timestamp = hrt_absolute_time();
            _actuators_0_pub.publish(actuators);
        //     PX4_WARN("-----------11111----------");
        }else {
        //     PX4_WARN("-----------2222----------");
        _attitude_control.resetESO();
        _attitude_control.resetPresetTraj();


            if (_v_control_mode.flag_control_termination_enabled) {
                    if (!_vehicle_status.is_vtol) {
                             // publish actuator controls
                            actuator_controls_s actuators{};
                            actuators.timestamp = hrt_absolute_time();
                            _actuators_0_pub.publish(actuators);
                    }
                }
        }
        // reset yaw setpoint during transitions, tailsitter.cpp generates
        // attitude setpoint for the transition
        _reset_yaw_sp = !attitude_setpoint_generated || _landed || (_vtol && _vtol_in_transition_mode);
    }
    perf_end(_loop_perf);
}

int UserAttitudeControl::task_spawn(int argc, char *argv[])
{
    UserAttitudeControl *instance = new UserAttitudeControl();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }

    } else {
        PX4_ERR("alloc failed");
    }
    PX4_WARN("test is closing !\n");
    delete instance;
    _object.store(nullptr);
    _task_id = -1;
    return PX4_ERROR;
}

int UserAttitudeControl::print_status()
{
    perf_print_counter(_loop_perf);
    return 0;
}

int UserAttitudeControl::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int UserAttitudeControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
                ### Description
                Example of a simple module running out of a work queue.

                )DESCR_STR");

    PRINT_MODULE_USAGE_NAME("usr_att_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}


extern "C" __EXPORT int usr_att_control_main(int argc, char *argv[])
{
    return UserAttitudeControl::main(argc, argv);
}


