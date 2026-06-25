add_subdirectory(Att_control)

px4_add_module(
	MODULE modules__pregme_att_control_v1
	MAIN   pregme_att_control_v1
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		pregme_att_control.cpp
		pregme_att_control.hpp
	MODULE_CONFIG
		pregme_att_control_params.yaml
	DEPENDS
		Att_control
		mathlib
		px4_work_queue
		StickYaw
	)
