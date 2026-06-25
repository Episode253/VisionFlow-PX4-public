px4_add_module(
	MODULE  modules__pregme_pos_control_v1
	MAIN    pregme_pos_control_v1
	COMPILE_FLAGS
	SRCS
		ControlMath.cpp
		ControlMath.hpp
		PosControl.cpp
		PosControl.hpp
		Takeoff.cpp
		Takeoff.hpp
		UserPositionControl.cpp
		UserPositionControl.hpp
	MODULE_CONFIG
		pregme_pos_control_params.yaml
	DEPENDS
		px4_work_queue
		controllib
		geo
		SlewRate
)
