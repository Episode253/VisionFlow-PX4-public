add_subdirectory(init.d)
# TODO: make this configurable from the board config, or better combine
if("${PX4_BOARD}" MATCHES "sitl")
	add_subdirectory(init.d-posix)
endif()
