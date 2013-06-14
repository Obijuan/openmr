cmake_minimum_required (VERSION 2.6.0)
project (openrave-modular-robots-examples)


set(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package(OpenRAVE REQUIRED)

#------- Make sure the openmr plugin is installed!

#-- Get the openrave share directory
EXEC_PROGRAM("openrave-config --share-dir" OUTPUT_VARIABLE OpenRAVE_SHARE_DIR)

find_library(OPENMR openmr ${OpenRAVE_SHARE_DIR}/plugins)
message(STATUS "Looking for OpenMR... ${OPENMR}")

if (NOT OPENMR)
  message(SEND_ERROR "OPENMR not found")
endif()


#---- Now check the compiler
if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT COMPONENTS iostreams python thread)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} )
if( Boost_INCLUDE_DIRS )
  include_directories(${Boost_INCLUDE_DIRS})
endif()

link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})


#-- Examples for the servocontroller

add_executable(Test-servocontroller-1 Test-servocontroller-1.cpp TestBase.cpp)
set_target_properties(Test-servocontroller-1 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Test-servocontroller-1 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Test-servocontroller-1 ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_executable(Test-servocontroller-2 Test-servocontroller-2.cpp TestBase.cpp)
set_target_properties(Test-servocontroller-2 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Test-servocontroller-2 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Test-servocontroller-2 ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_executable(Test-servocontroller-3 Test-servocontroller-3.cpp TestBase.cpp)
set_target_properties(Test-servocontroller-3 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Test-servocontroller-3 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Test-servocontroller-3 ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_executable(Test-servocontroller-4 Test-servocontroller-4.cpp TestBase.cpp)
set_target_properties(Test-servocontroller-4 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Test-servocontroller-4 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Test-servocontroller-4 ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

#-- Examples for the sinoscontroller

add_executable(Test-sinoscontroller-1 Test-sinoscontroller-1.cpp TestBase.cpp)
set_target_properties(Test-sinoscontroller-1 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Test-sinoscontroller-1 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Test-sinoscontroller-1 ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_executable(Test-sinoscontroller-2 Test-sinoscontroller-2.cpp TestBase.cpp)
set_target_properties(Test-sinoscontroller-2 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Test-sinoscontroller-2 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Test-sinoscontroller-2 ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_executable(Minicube-I-1 Minicube-I-1.cpp TestBase.cpp)
set_target_properties(Minicube-I-1 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Minicube-I-1 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Minicube-I-1 ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_executable(Cube3-I Cube3-I.cpp TestBase.cpp)
set_target_properties(Cube3-I PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Cube3-I PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Cube3-I ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_executable(Cube4-I Cube4-I.cpp TestBase.cpp)
set_target_properties(Cube4-I PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Cube4-I PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Cube4-I ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_executable(Cube5-I Cube5-I.cpp TestBase.cpp)
set_target_properties(Cube5-I PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Cube5-I PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Cube5-I ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})


add_executable(Minicube-II-1 Minicube-II-1.cpp TestBase.cpp)
set_target_properties(Minicube-II-1 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Minicube-II-1 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Minicube-II-1 ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_executable(snake snake.cpp TestBase.cpp)
set_target_properties(snake PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(snake PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(snake ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_executable(test test.cpp TestBase.cpp)
set_target_properties(test PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(test PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(test ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})



