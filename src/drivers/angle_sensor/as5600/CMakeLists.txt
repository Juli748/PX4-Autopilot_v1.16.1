px4_add_module(
	MODULE drivers__angle_sensor__as5600
	MAIN as5600
	SRCS
		AS5600.cpp
		AS5600.hpp
		AS5600_main.cpp
	DEPENDS
		px4_work_queue
	)
