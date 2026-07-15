px4_add_module(
	MODULE modules__pregme_pos_control_v2
	MAIN pregme_pos_control_v2
	COMPILE_FLAGS
	SRCS
		ControlMath.cpp
		ControlMath.hpp
		PosControl.cpp
		PosControl.hpp
		Takeoff.cpp
		Takeoff.hpp
		PregmePositionControl.cpp
		PregmePositionControl.hpp
	MODULE_CONFIG
		pregme_pos_control_params_zh.yaml
		# pregme_pos_control_params_en.yaml
	DEPENDS
		px4_work_queue
		controllib
		geo
		SlewRate
)
