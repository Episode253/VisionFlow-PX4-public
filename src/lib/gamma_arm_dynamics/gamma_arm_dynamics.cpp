/**
 * @file gamma_arm_dynamics.cpp
 */

#include "gamma_arm_dynamics.hpp"
#include <math.h>

namespace gamma_arm
{

ArmUavParam makeDefaultParam()
{
	ArmUavParam p;

	// ── 无人机 (swan_uav_v2/model.sdf base_link inertial) ──
	p.m_uav = 7.8874f;
	p.pc_uav = matrix::Vector3f(-0.0751f, 0.f, -0.0322f);
	{
		auto &I = p.I_uav;
		I(0,0)=0.3655f; I(0,1)=0.f;     I(0,2)=0.f;
		I(1,0)=0.f;     I(1,1)=0.1895f; I(1,2)=0.f;
		I(2,0)=0.f;     I(2,1)=0.f;     I(2,2)=0.5488f;
	}

	// <pose>0.405 -0.015 -0.0295 -2.6443 0 0</pose>
	p.p_mount = matrix::Vector3f(0.405f, 0.f, 0.f);
	{
		auto &R = p.R_mount;
		R.setZero();
		R(0,0)=0.0f;  R(0,1)= 0.0f;  R(0,2)= 1.0f;
		R(1,0)=0.0f;  R(1,1)=-1.0f;  R(1,2)= 0.0f;
		R(2,0)=1.0f;  R(2,1)= 0.0f;  R(2,2)= 0.0f;
	}

	// ── DH (零位: A=-π, B=C=D=E=F=0) ──
	// p.dh[0] = {0.06407f, -kPi/2.f,   0.10429f,  0.f};
	// p.dh[1] = {0.24873f,  0.f,       0.02305f,  0.f};
	// p.dh[2] = {0.06301f,  kPi/2.f,  -0.025f,    0.f};
	// p.dh[3] = {0.f,       kPi/2.f,   0.165f,    kPi/2.f};
	// p.dh[4] = {0.f,       kPi/2.f,  -0.0015f,   kPi};
	// p.dh[5] = {0.f,       0.f,       0.084f,    kPi};

	p.dh[0] = {0.06407f, -kPi/2.f,   0.10429f,  kPi};
	p.dh[1] = {0.24873f,  0.f,       0.02305f,  0.f};
	p.dh[2] = {0.06301f,  kPi/2.f,  -0.025f,    0.f};
	p.dh[3] = {0.f,       kPi/2.f,   0.165f,    kPi/2.f};
	p.dh[4] = {0.f,       kPi/2.f,  -0.0015f,   kPi};
	p.dh[5] = {0.f,       0.f,       0.084f,    kPi};

	// ── base_link ──
	p.base_link.m  = 0.361f;
	p.base_link.pc = matrix::Vector3f(0.f, 0.f, 0.026f);
	{
		auto &I = p.base_link.I;
		I(0,0)=0.00014566f; I(0,1)=0.f; I(0,2)=0.f;
		I(1,0)=0.f; I(1,1)=0.00015872f; I(1,2)=0.f;
		I(2,0)=0.f; I(2,1)=0.f; I(2,2)=0.00015872f;
	}

	// ── A_Link ──
	p.links[0].m  = 0.482f;
	p.links[0].pc = matrix::Vector3f(-0.0105f, 0.0067f, -0.0074f);
	{
		auto &I = p.links[0].I;
		I(0,0)= 0.00048916f; I(0,1)=-3.127e-05f;  I(0,2)= 0.00014844f;
		I(1,0)=-3.127e-05f;  I(1,1)= 0.00049299f; I(1,2)=-5.578e-05f;
		I(2,0)= 0.00014844f; I(2,1)=-5.578e-05f;  I(2,2)= 0.0003462f;
	}

	// ── B_Link ──
	p.links[1].m  = 0.481f;
	p.links[1].pc = matrix::Vector3f(-0.0307f, 0.f, -0.0322f);
	{
		auto &I = p.links[1].I;
		I(0,0)= 0.0015459f;  I(0,1)= 0.00021428f; I(0,2)=-0.0011607f;
		I(1,0)= 0.00021428f; I(1,1)= 0.0025514f;  I(1,2)=-0.00023947f;
		I(2,0)=-0.0011607f;  I(2,1)=-0.00023947f; I(2,2)= 0.0012874f;
	}

	// ── C_Link ──
	p.links[2].m  = 0.239f;
	p.links[2].pc = matrix::Vector3f(-0.0055f, 0.0017f, -0.0066f);
	{
		auto &I = p.links[2].I;
		I(0,0)= 0.00014499f; I(0,1)=-3.1e-07f;  I(0,2)=-4.26e-06f;
		I(1,0)=-3.1e-07f;    I(1,1)= 0.0001552f; I(1,2)= 1.928e-05f;
		I(2,0)=-4.26e-06f;   I(2,1)= 1.928e-05f; I(2,2)= 0.00010724f;
	}

	// ── D_Link ──
	p.links[3].m  = 0.262f;
	p.links[3].pc = matrix::Vector3f(-7.6e-05f, -0.0176f, 0.0043f);
	{
		auto &I = p.links[3].I;
		I(0,0)= 0.00010245f; I(0,1)=-1.568e-05f; I(0,2)= 3.8e-07f;
		I(1,0)=-1.568e-05f;  I(1,1)= 0.00052076f; I(1,2)= 3.2e-07f;
		I(2,0)= 3.8e-07f;    I(2,1)= 3.2e-07f;    I(2,2)= 0.0005369f;
	}

	// ── E_Link ──
	p.links[4].m  = 0.042397;
	p.links[4].pc = matrix::Vector3f(3.7e-05f, 0.0035f, 0.0582f);
	{
		auto &I = p.links[4].I;
		I(0,0)=6.544e-05f; I(0,1)=0.f;      I(0,2)=0.f;
		I(1,0)=0.f;        I(1,1)=5.714e-05f; I(1,2)=1.564e-05f;
		I(2,0)=0.f;        I(2,1)=1.564e-05f; I(2,2)=2.005e-05f;
	}

	// ── F_Link ──
	p.links[5].m  = 0.234f;
	p.links[5].pc = matrix::Vector3f(-4.6e-05f, 0.000103f, -0.024394f);
	{
		auto &I = p.links[5].I;
		I(0,0)=8.297e-05f; I(0,1)=3e-08f;     I(0,2)= 1.6e-07f;
		I(1,0)=3e-08f;     I(1,1)=8.274e-05f;  I(1,2)=-3.1e-07f;
		I(2,0)=1.6e-07f;   I(2,1)=-3.1e-07f;   I(2,2)= 6.854e-05f;
	}

	return p;
}

static const ArmUavParam kDefaultParam = makeDefaultParam();
const ArmUavParam &kDefaultModelParam = kDefaultParam;

static matrix::SquareMatrix<float, 4> makeDhTransform(const DhParam &dh, float q_i)
{
	const float t = dh.theta + q_i;
	const float ct = cosf(t), st = sinf(t);
	const float ca = cosf(dh.alpha), sa = sinf(dh.alpha);

	matrix::SquareMatrix<float, 4> T;
	T(0,0)=ct;  T(0,1)=-st*ca; T(0,2)= st*sa; T(0,3)=dh.a*ct;
	T(1,0)=st;  T(1,1)= ct*ca; T(1,2)=-ct*sa; T(1,3)=dh.a*st;
	T(2,0)=0;   T(2,1)= sa;    T(2,2)= ca;    T(2,3)=dh.d;
	T(3,0)=0;   T(3,1)= 0;     T(3,2)= 0;     T(3,3)=1.f;
	return T;
}

static matrix::SquareMatrix<float, 3> rot33(const matrix::SquareMatrix<float, 4> &T)
{
	matrix::SquareMatrix<float, 3> R;
	R(0,0)=T(0,0); R(0,1)=T(0,1); R(0,2)=T(0,2);
	R(1,0)=T(1,0); R(1,1)=T(1,1); R(1,2)=T(1,2);
	R(2,0)=T(2,0); R(2,1)=T(2,1); R(2,2)=T(2,2);
	return R;
}

static matrix::Vector3f trans(const matrix::SquareMatrix<float, 4> &T)
{
	return matrix::Vector3f(T(0,3), T(1,3), T(2,3));
}

static matrix::Vector3f toBody(const matrix::Vector3f &p_arm,
			       const ArmUavParam &param)
{
	return param.p_mount + param.R_mount * p_arm;
}

ComStateInBody computeComStateInBody(const float q[6], const ArmUavParam &param)
{
	constexpr float kMassEps = 1e-9f;
	ComStateInBody s;

	s.p_c_uav_B = param.pc_uav;

	// 正运动学: DH 连乘
	matrix::SquareMatrix<float, 4> T[kArmLinkNum];
	{
		matrix::SquareMatrix<float, 4> acc;
		acc.setIdentity();
		for (int i = 0; i < kArmLinkNum; ++i) {
			acc = acc * makeDhTransform(param.dh[i], q[i]);
			T[i] = acc;
		}
	}

	// base_link
	s.p_c_base_link_B = toBody(param.base_link.pc, param);
	float m_arm = param.base_link.m;
	matrix::Vector3f moment = param.base_link.m * s.p_c_base_link_B;

	// 连杆 A~F
	for (int i = 0; i < kArmLinkNum; ++i) {
		const matrix::Vector3f p_in_arm = trans(T[i]) + rot33(T[i]) * param.links[i].pc;
		s.p_c_links_B[i] = toBody(p_in_arm, param);

		m_arm  += param.links[i].m;
		moment += param.links[i].m * s.p_c_links_B[i];
	}

	// 机械臂质心
	s.m_arm = m_arm;
	if (m_arm > kMassEps) {
		s.p_C_arm_B = moment / m_arm;
	}

	// 系统总质心
	s.m_total = param.m_uav + m_arm;
	if (s.m_total > kMassEps) {
		s.p_C_B = (param.m_uav * s.p_c_uav_B + moment) / s.m_total;
	}

	return s;
}

} // namespace gamma_arm
