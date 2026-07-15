add_subdirectory(Att_control)

px4_add_module(
	MODULE modules__pregme_att_control
	MAIN   pregme_att_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		pregme_att_control.cpp
		pregme_att_control.hpp
	MODULE_CONFIG
		pregme_att_control_params_zh.yaml
		# pregme_att_control_params_en.yaml
	DEPENDS
		Att_control
		gamma_arm_dynamics
		mathlib
		px4_work_queue
		StickYaw
	)
