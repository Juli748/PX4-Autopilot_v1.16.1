px4_add_module(
	MODULE drivers__angle_sensor__aoa_vane_as5600
	MAIN aoa_vane_as5600
	SRCS
		AoaVaneAS5600.cpp
		AoaVaneAS5600.hpp
		AoaVaneAS5600_main.cpp
		parameters.c
	DEPENDS
		px4_work_queue
	)
