#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pymavlink import mavutil
import time
import math

SERIAL_PORT = "/dev/ttyACM0"
BAUD = 2000000

LAT_DEG = 47.397742
LON_DEG = 8.545594
ALT_M = 488.0

LAT = int(LAT_DEG * 1e7)      # degE7
LON = int(LON_DEG * 1e7)      # degE7
ALT = int(ALT_M * 1000)       # mm

HIL_SENSOR_HZ = 250.0
HIL_GPS_HZ = 10.0
HIL_STATE_HZ = 50.0
HEARTBEAT_HZ = 1.0
ACTUATOR_PRINT_HZ = 2.0

HIL_SENSOR_PERIOD = 1.0 / HIL_SENSOR_HZ
HIL_GPS_PERIOD = 1.0 / HIL_GPS_HZ
HIL_STATE_PERIOD = 1.0 / HIL_STATE_HZ
HEARTBEAT_PERIOD = 1.0 / HEARTBEAT_HZ
ACTUATOR_PRINT_PERIOD = 1.0 / ACTUATOR_PRINT_HZ


def now_us():
    return int(time.time() * 1e6)


def pressure_from_altitude_m(alt_m):
    return 1013.25 * math.pow(1.0 - alt_m / 44330.0, 5.255)


def send_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )


def send_hil_sensor(master):

    time_usec = now_us()

    fields_updated = 0x1FFF

    abs_pressure_hpa = pressure_from_altitude_m(ALT_M)
    pressure_alt_m = ALT_M

    master.mav.hil_sensor_send(
        time_usec,

        # acceleration, body frame, m/s^2
        0.0,        # xacc
        0.0,        # yacc
        -9.80665,   # zacc

        # gyro, rad/s
        0.0,        # xgyro
        0.0,        # ygyro
        0.0,        # zgyro

        # magnetometer, gauss
        0.2,        # xmag
        0.0,        # ymag
        0.4,        # zmag

        # pressure
        abs_pressure_hpa,    # abs_pressure, hPa
        0.0,                 # diff_pressure
        pressure_alt_m,      # pressure_alt, m
        25.0,                # temperature, degC

        fields_updated,
        0                    # sensor id
    )


def send_hil_gps(master):

    time_usec = now_us()

    fix_type = 3                 # 3 = 3D Fix
    eph = 80                     # cm
    epv = 120                    # cm

    vel = 0                      # cm/s
    vn = 0                       # cm/s
    ve = 0                       # cm/s
    vd = 0                       # cm/s

    cog = 0                      # cdeg
    satellites_visible = 12

    master.mav.hil_gps_send(
        time_usec,
        fix_type,
        LAT,
        LON,
        ALT,
        eph,
        epv,
        vel,
        vn,
        ve,
        vd,
        cog,
        satellites_visible
    )


def send_hil_state_quaternion(master):

    time_usec = now_us()

    q = [1.0, 0.0, 0.0, 0.0]

    rollspeed = 0.0
    pitchspeed = 0.0
    yawspeed = 0.0

    vx = 0
    vy = 0
    vz = 0

    ind_airspeed = 0
    true_airspeed = 0

    xacc = 0
    yacc = 0
    zacc = -1000

    master.mav.hil_state_quaternion_send(
        time_usec,
        q,
        rollspeed,
        pitchspeed,
        yawspeed,
        LAT,
        LON,
        ALT,
        vx,
        vy,
        vz,
        ind_airspeed,
        true_airspeed,
        xacc,
        yacc,
        zacc
    )


def read_actuator_controls(master):

    while True:
        msg = master.recv_match(
            type="HIL_ACTUATOR_CONTROLS",
            blocking=False
        )

        if msg is None:
            break

        controls = msg.controls

        print(
            "[HIL_ACTUATOR_CONTROLS] "
            f"mode={msg.mode} "
            f"c0={controls[0]: .4f}, "
            f"c1={controls[1]: .4f}, "
            f"c2={controls[2]: .4f}, "
            f"c3={controls[3]: .4f}"
        )


def main():
    print(f"[INFO] Connecting to PX4 on {SERIAL_PORT} @ {BAUD} ...")

    master = mavutil.mavlink_connection(
        SERIAL_PORT,
        baud=BAUD,
        source_system=2,
        source_component=1,
        force_connected=True
    )

    print("[INFO] Waiting for PX4 heartbeat...")
    master.wait_heartbeat()

    print(
        f"[OK] Heartbeat from system {master.target_system}, "
        f"component {master.target_component}"
    )

    print("[INFO] Static HITL test started.")
    print(f"[INFO] GPS position: lat={LAT_DEG}, lon={LON_DEG}, alt={ALT_M} m")
    print(f"[INFO] Baro pressure: {pressure_from_altitude_m(ALT_M):.2f} hPa")
    print("[INFO] Sending:")
    print(f"       HIL_SENSOR           @ {HIL_SENSOR_HZ:.0f} Hz")
    print(f"       HIL_GPS              @ {HIL_GPS_HZ:.0f} Hz")
    print(f"       HIL_STATE_QUATERNION @ {HIL_STATE_HZ:.0f} Hz")
    print("[INFO] PX4 NSH check commands:")
    print("       listener sensor_gps")
    print("       listener sensor_baro")
    print("       listener vehicle_attitude")
    print("       listener vehicle_local_position")
    print("       listener estimator_status")
    print("       commander check")
    print("       pwm_out_sim status")
    print("[INFO] Press Ctrl+C to stop.")

    last_hil_sensor_time = 0.0
    last_hil_gps_time = 0.0
    last_hil_state_time = 0.0
    last_heartbeat_time = 0.0
    last_actuator_print_time = 0.0

    try:
        while True:
            t = time.monotonic()

            if t - last_heartbeat_time >= HEARTBEAT_PERIOD:
                send_heartbeat(master)
                last_heartbeat_time = t

            if t - last_hil_sensor_time >= HIL_SENSOR_PERIOD:
                send_hil_sensor(master)
                last_hil_sensor_time = t

            if t - last_hil_gps_time >= HIL_GPS_PERIOD:
                send_hil_gps(master)
                last_hil_gps_time = t

            if t - last_hil_state_time >= HIL_STATE_PERIOD:
                send_hil_state_quaternion(master)
                last_hil_state_time = t

            if t - last_actuator_print_time >= ACTUATOR_PRINT_PERIOD:
                read_actuator_controls(master)
                last_actuator_print_time = t

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped.")


if __name__ == "__main__":
    main()
