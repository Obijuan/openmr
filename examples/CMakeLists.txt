set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Make sure the openmr plugin is installed!
# Get the openrave share directory
EXEC_PROGRAM("openrave-config --share-dir" OUTPUT_VARIABLE OpenRAVE_SHARE_DIR)

find_library(OPENMR openmr ${OpenRAVE_SHARE_DIR}/plugins)
message(STATUS "Looking for OpenMR... ${OPENMR}")

if (NOT OPENMR)
  message(SEND_ERROR "OPENMR not found")
endif()


# Include and link directories
include_directories(${OpenRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} )
if( Boost_INCLUDE_DIRS )
  include_directories(${Boost_INCLUDE_DIRS})
endif()

link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})


# Compile TestBase as static library:
add_library( TESTBASE TestBase.cpp)
set_target_properties( TESTBASE PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties( TESTBASE PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries( TESTBASE ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY})

# Macro for compiling the examples
macro( build_openmr_example name)
	add_executable( ${name} ${name}.cpp)
	set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
	set_target_properties(${name} PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
	target_link_libraries(${name} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} TESTBASE)
endmacro( build_openmr_example)


# Examples for the servocontroller
build_openmr_example( Test-servocontroller-1 )
build_openmr_example( Test-servocontroller-2 )
build_openmr_example( Test-servocontroller-3 )
build_openmr_example( Test-servocontroller-4 )


#-- Examples for the sinoscontroller
build_openmr_example( Test-sinoscontroller-1 )
build_openmr_example( Test-sinoscontroller-2 )

# Examples with snakes
build_openmr_example( Minicube-I-1 )
build_openmr_example( Cube3-I )
build_openmr_example( Cube4-I )
build_openmr_example( Cube5-I )
build_openmr_example( Minicube-II-1 )
build_openmr_example( Snake5-I )
build_openmr_example( Test-plotting )


