add_subdirectory(init.d)

if("${PX4_BOARD}" MATCHES "sitl")
	add_subdirectory(init.d-posix)
endif()
