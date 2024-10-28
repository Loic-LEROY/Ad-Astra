# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles\\ad-astra_autogen.dir\\AutogenUsed.txt"
  "CMakeFiles\\ad-astra_autogen.dir\\ParseCache.txt"
  "ad-astra_autogen"
  )
endif()
