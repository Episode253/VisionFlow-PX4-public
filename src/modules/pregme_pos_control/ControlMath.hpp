#pragma once

#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>

namespace ControlMath
{

void thrustToAttitude(const matrix::Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp);

void limitTilt(matrix::Vector3f &body_unit, const matrix::Vector3f &world_unit, const float max_angle);

void bodyzToAttitude(matrix::Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp);

matrix::Vector2f constrainXY(const matrix::Vector2f &v0, const matrix::Vector2f &v1, const float &max);

bool cross_sphere_line(const matrix::Vector3f &sphere_c, const float sphere_r,
		       const matrix::Vector3f &line_a, const matrix::Vector3f &line_b, matrix::Vector3f &res);

void addIfNotNan(float &setpoint, const float addition);

void addIfNotNanVector3f(matrix::Vector3f &setpoint, const matrix::Vector3f &addition);

void setZeroIfNanVector3f(matrix::Vector3f &vector);

void setPositionIfPositionIsNan(matrix::Vector3f &vel_sp, matrix::Vector3f &pos_sp, float dt);

} // namespace
