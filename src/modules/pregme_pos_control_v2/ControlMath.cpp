#include "ControlMath.hpp"

#include <float.h>
#include <cmath>

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

using namespace matrix;

namespace ControlMath
{

void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
	att_sp.thrust_body[2] = -thr_sp.length();
}

void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, const float max_angle)
{
	const float dot_product_unit = body_unit.dot(world_unit);
	float angle = acosf(math::constrain(dot_product_unit, -1.f, 1.f));
	angle = math::min(angle, max_angle);

	Vector3f rejection = body_unit - (dot_product_unit * world_unit);

	if (rejection.norm_squared() < FLT_EPSILON) {
		rejection(0) = 1.f;
	}

	body_unit = cosf(angle) * world_unit + sinf(angle) * rejection.unit();
}

void bodyzToAttitude(Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	if (body_z.norm_squared() < FLT_EPSILON) {
		body_z(2) = 1.f;
	}

	body_z.normalize();

	const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};
	Vector3f body_x = y_C % body_z;

	if (body_z(2) < 0.f) {
		body_x = -body_x;
	}

	if (fabsf(body_z(2)) < 1e-6f) {
		body_x.zero();
		body_x(2) = 1.f;
	}

	body_x.normalize();

	const Vector3f body_y = body_z % body_x;

	Dcmf R_sp;
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);

}

Vector2f constrainXY(const Vector2f &v0, const Vector2f &v1, const float &max)
{
	const Vector2f sum{v0 + v1};

	if (sum.norm() <= max) {
		return sum;

	} else if (v0.length() >= max) {
		return v0.normalized() * max;

	} else if (fabsf(Vector2f(v1 - v0).norm()) < 0.001f) {
		return v0.normalized() * max;

	} else if (v0.length() < 0.001f) {
		return v1.normalized() * max;

	} else {
		Vector2f u1 = v1.normalized();
		const float m = u1.dot(v0);
		const float c = v0.dot(v0) - max * max;
		const float s = -m + sqrtf(m * m - c);
		return v0 + u1 * s;
	}
}

bool cross_sphere_line(const Vector3f &sphere_c, const float sphere_r,
		       const Vector3f &line_a, const Vector3f &line_b, Vector3f &res)
{
	Vector3f ab_norm = line_b - line_a;

	if (ab_norm.length() < 0.01f) {
		return true;
	}

	ab_norm.normalize();
	Vector3f d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	const float cd_len = (sphere_c - d).length();

	if (sphere_r > cd_len) {
		const float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			res = line_b;

		} else {
			res = d + ab_norm * dx_len;
		}

		return true;

	} else {
		res = d;

		if ((sphere_c - line_a) * ab_norm < 0.0f) {
			res = line_a;
		}

		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			res = line_b;
		}

		return false;
	}
}

void addIfNotNan(float &setpoint, const float addition)
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {
		setpoint = addition;
	}
}

void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		addIfNotNan(setpoint(i), addition(i));
	}
}

void setZeroIfNanVector3f(Vector3f &vector)
{
	addIfNotNanVector3f(vector, Vector3f());
}

void setPositionIfPositionIsNan(Vector3f &vel_sp, Vector3f &pos_sp, float dt)
{
	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(pos_sp(i)) && PX4_ISFINITE(vel_sp(i))) {
			pos_sp(i) = pos_sp(i) + vel_sp(i) * dt;
		}
	}
}

} // namespace
