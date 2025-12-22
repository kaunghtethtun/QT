# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/netplan_assign_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/netplan_assign_autogen.dir/ParseCache.txt"
  "netplan_assign_autogen"
  )
endif()
