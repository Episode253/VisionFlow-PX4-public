add_subdirectory(Att_control)

px4_add_module(
	MODULE modules__usr_att_control
	MAIN   usr_att_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		usr_att_control.cpp
		usr_att_control.hpp
	MODULE_CONFIG
		usr_att_control_params.yaml
	DEPENDS
		Att_control
		mathlib
		px4_work_queue
		StickYaw
	)
