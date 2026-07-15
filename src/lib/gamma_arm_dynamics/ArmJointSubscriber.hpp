#pragma once

/**
 * @file ArmJointSubscriber.hpp
 *
 * 机械臂关节角订阅器 (单例)
 *
 * 职责:
 *  - 从 MAVLink DEBUG_FLOAT_ARRAY (name="arm_joint") 读取 6 个关节角 [rad]
 *  - 调用 gamma_arm::computeComStateInBody() 计算系统质心
 *  - 以线程安全方式缓存最新结果, 供姿态环 / 位置环 / Logger 无锁读取
 *
 * 使用:
 *   #include "ArmJointSubscriber.hpp"
 *   ArmJointSubscriber *arm = ArmJointSubscriber::instance();
 *   arm->update();                              // 在控制循环中调用
 *   Vector3f com = arm->getSystemCom();         // 系统总质心 (机体系 NED)
 *   Vector3f arm_com = arm->getArmCom();        // 机械臂质心
 *   float total_mass = arm->getTotalMass();     // 系统总质量
 */

#include <matrix/matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/debug_array.h>

#include <lib/gamma_arm_dynamics/gamma_arm_dynamics.hpp>

class ArmJointSubscriber
{
public:
	static ArmJointSubscriber *instance();

	~ArmJointSubscriber() = default;

	/** 轮询 MAVLink 消息, 有新关节角时重新计算质心 (非阻塞). */
	void update();

	matrix::Vector3f getSystemCom()  const { return _system_com_B; }
	matrix::Vector3f getArmCom()     const { return _arm_com_B; }
	float           getTotalMass()   const { return _total_mass; }
	float           getArmMass()     const { return _arm_mass; }
	const float    *getJointAngles() const { return _arm_q; }

private:
	ArmJointSubscriber();

	uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};

	matrix::Vector3f _system_com_B{};
	matrix::Vector3f _arm_com_B{};
	float _total_mass{0.f};
	float _arm_mass{0.f};
	float _arm_q[6]{};
};
