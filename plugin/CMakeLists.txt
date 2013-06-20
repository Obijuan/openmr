# Include and link directories
include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

# Look for the share directory
EXEC_PROGRAM("openrave-config --share-dir" OUTPUT_VARIABLE OpenRAVE_SHARE_DIR)
message(STATUS "Openrave Share directory: ${OpenRAVE_SHARE_DIR}")


# OpenMR
add_library(openmr SHARED openmr.cpp servocontroller.h sinoscontroller.h)
set_target_properties(openmr PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(openmr ${OpenRAVE_LIBRARIES})
install(TARGETS openmr DESTINATION ${OpenRAVE_SHARE_DIR}/plugins/)

