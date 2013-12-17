remake_project_set(CAN_INTERFACE cpc CACHE STRING
  "The motor control's CAN interface as supported by libcan [cpc|serial].")

remake_project_get(CAN_INTERFACE)
remake_find_library(
  can can.h
  PACKAGE libcan
)
remake_find_library(
  can-${CAN_INTERFACE} can.h
  PACKAGE libcan
)
remake_var_name(CAN_LIBRARY_VAR can-${CAN_INTERFACE} LIBRARY)

remake_find_library(
  epos epos.h
  PACKAGE libepos
)	

remake_add_library(
	rover PREFIX OFF librover.cc rover.cc core.cc controller.cc flipper.cc track.cc
	LINK ${CAN_LIBRARY} ${${CAN_LIBRARY_VAR}} ${EPOS_LIBRARY}
)
remake_add_headers()
