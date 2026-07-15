#pragma once

/**
 * @file gamma_arm_dynamics.hpp
 */

#include <array>
#include <math.h>
#include <matrix/matrix/math.hpp>

namespace gamma_arm
{

static constexpr int kJointNum  = 6;
static constexpr int kArmLinkNum = 6;
static constexpr float kPi      = 3.14159265358979323846f;

struct DhParam {
	float a     = 0.f;
	float alpha = 0.f;
	float d     = 0.f;
	float theta = 0.f;
};

struct BodyParam {
	float          m  = 0.f;
	matrix::Vector3f pc{};
	matrix::SquareMatrix<float, 3> I{};
};

struct ArmUavParam {
	float          m_uav   = 0.f;
	matrix::Vector3f pc_uav{};
	matrix::SquareMatrix<float, 3> I_uav{};

	matrix::Vector3f p_mount{};
	matrix::SquareMatrix<float, 3> R_mount;

	BodyParam base_link{};
	std::array<BodyParam, kArmLinkNum> links{};
	std::array<DhParam, kJointNum> dh{};
};

struct ComStateInBody {
	matrix::Vector3f p_c_uav_B{};
	matrix::Vector3f p_c_base_link_B{};
	std::array<matrix::Vector3f, kArmLinkNum> p_c_links_B{};
	matrix::Vector3f p_C_arm_B{};
	matrix::Vector3f p_C_B{};
	float m_arm   = 0.f;
	float m_total = 0.f;
};

// ─── API ───

ArmUavParam makeDefaultParam();

extern const ArmUavParam &kDefaultModelParam;

ComStateInBody computeComStateInBody(const float q[6],
				     const ArmUavParam &param);

} // namespace gamma_arm
