# Copyright (c) 2021 Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_COMMONLIBRARY_ENABLE)

zephyr_library_sources(
 	${CMAKE_CURRENT_SOURCE_DIR}/algorithmlib/pid.c                  
	${CMAKE_CURRENT_SOURCE_DIR}/algorithmlib/filter.c               
	${CMAKE_CURRENT_SOURCE_DIR}/algorithmlib/s_posi_planning.c      
	${CMAKE_CURRENT_SOURCE_DIR}/algorithmlib/s_trajectory_planning.c
	${CMAKE_CURRENT_SOURCE_DIR}/algorithmlib/fault_monitoring_module.c
	${CMAKE_CURRENT_SOURCE_DIR}/statemachine/statemachine.c
)

zephyr_include_directories(
    algorithmlib
    statemachine
)

endif()