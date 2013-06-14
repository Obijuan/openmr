cmake_minimum_required (VERSION 2.6.0)
project (openmr)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package(OpenRAVE REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT COMPONENTS iostreams python thread)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} )
if( Boost_INCLUDE_DIRS )
  include_directories(${Boost_INCLUDE_DIRS})
endif()

link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

#-- Get the openrave share directory
EXEC_PROGRAM("openrave-config --share-dir" OUTPUT_VARIABLE OpenRAVE_SHARE_DIR)

message(STATUS "Openrave Share directory: ${OpenRAVE_SHARE_DIR}")

add_library(openmr SHARED openmrmain.cpp  plugindefs.h servocontroller.h sinoscontroller.h)
set_target_properties(openmr PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}")
set_target_properties(openmr PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(openmr ${OpenRAVE_LIBRARIES})
install(TARGETS openmr DESTINATION ${OpenRAVE_SHARE_DIR}/plugins)



