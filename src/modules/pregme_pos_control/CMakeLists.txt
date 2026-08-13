px4_add_module(
	MODULE modules__pregme_pos_control
	MAIN pregme_pos_control
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
		# pregme_pos_control_params_zh.yaml
		# pregme_pos_control_params_en.yaml
		pregme_pos_control_params_gazebo.yaml
	DEPENDS
	        gamma_arm_dynamics
		px4_work_queue
		controllib
		geo
		SlewRate
)
