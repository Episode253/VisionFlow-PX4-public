px4_add_module(
	MODULE modules__pregme_pos_control
	MAIN pregme_pos_control
	COMPILE_FLAGS
	SRCS
		ControlMath.cpp
		PosControl.cpp
		Takeoff.cpp
		PregmePositionControl.cpp
	MODULE_CONFIG
		pregme_pos_control_params_zh.yaml
		# pregme_pos_control_params_en.yaml
	DEPENDS
		px4_work_queue
		controllib
		geo
		SlewRate
)
