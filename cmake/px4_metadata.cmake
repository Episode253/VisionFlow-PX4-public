#=============================================================================
#
#	Defined functions in this file
#
# 	utility functions
#
#		* px4_generate_airframes_xml
#

#=============================================================================
#
#	px4_generate_airframes_xml
#
#	Generates airframes.xml
#
#	Usage:
#		px4_generate_airframes_xml(OUT <airframe-xml-file>)
#
#	Input:
#		XML : the airframes.xml file
#		BOARD : the board
#
#	Output:
#		OUT	: the generated source files
#
#	Example:
#		px4_generate_airframes_xml(OUT airframes.xml)
#
function(px4_generate_airframes_xml)
	px4_parse_function_args(
		NAME px4_generate_airframes_xml
		ONE_VALUE BOARD
		REQUIRED BOARD
		ARGN ${ARGN})

	add_custom_command(OUTPUT ${PX4_BINARY_DIR}/airframes.xml
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/python_scripts/px_process_airframes.py
			--airframes-path ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/init.d
			--board CONFIG_ARCH_BOARD_${PX4_BOARD}
			--xml ${PX4_BINARY_DIR}/airframes.xml
		DEPENDS ${PX4_SOURCE_DIR}/Tools/python_scripts/px_process_airframes.py
		COMMENT "Creating airframes.xml"
		)
	add_custom_target(airframes_xml DEPENDS ${PX4_BINARY_DIR}/airframes.xml)
endfunction()
