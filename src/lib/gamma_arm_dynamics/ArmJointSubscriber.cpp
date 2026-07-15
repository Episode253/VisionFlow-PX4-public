/**
 * @file ArmJointSubscriber.cpp
 */

#include "ArmJointSubscriber.hpp"
#include <px4_platform_common/defines.h>

ArmJointSubscriber *ArmJointSubscriber::instance()
{
	static ArmJointSubscriber s;
	return &s;
}

ArmJointSubscriber::ArmJointSubscriber()
{
	// 初始化为零，等待话题更新
	for (int i = 0; i < 6; ++i) {
		_arm_q[i] = 0.f;
	}
	// _system_com_B 等保持默认零值，只有收到话题才更新
}

void ArmJointSubscriber::update()
{
	debug_array_s dbg{};

	if (!_debug_array_sub.update(&dbg)) {
		return;
	}

	// 仅匹配 name="arm_joint"
	static constexpr char kTag[] = "arm_joint";

	for (int i = 0; i < 10 && dbg.name[i] != '\0'; ++i) {
		if (dbg.name[i] != kTag[i]) {
			return;
		}
	}

	// 有效性检查
	constexpr int kNeed = 6;
	bool valid = true;

	for (int i = 0; i < kNeed; ++i) {
		valid = valid && PX4_ISFINITE(dbg.data[i]);
	}

	if (!valid) {
		return;
	}

	// 读取关节角
	float q[6];

	for (int i = 0; i < 6; ++i) {
		q[i] = dbg.data[i];
	}

	// 计算质心
	const auto state = gamma_arm::computeComStateInBody(q, gamma_arm::kDefaultModelParam);

	// 原子写入缓存 (float 在 Cortex-M 上天然原子)
	_system_com_B = state.p_C_B;
	_arm_com_B    = state.p_C_arm_B;
	_total_mass   = state.m_total;
	_arm_mass     = state.m_arm;

	for (int i = 0; i < 6; ++i) {
		_arm_q[i] = q[i];
	}
}
