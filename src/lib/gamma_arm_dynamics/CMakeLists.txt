px4_add_library(gamma_arm_dynamics
	gamma_arm_dynamics.cpp
	gamma_arm_dynamics.hpp
	ArmJointSubscriber.cpp
	ArmJointSubscriber.hpp
)

target_compile_options(gamma_arm_dynamics PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(gamma_arm_dynamics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
