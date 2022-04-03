cmake_minimum_required(VERSION 3.0.2)

project(husky_inekf)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed -DBOOST_LOG_DYN_LINK -lboost_log -lpthread -lboost_timer" ) #-DBOOST_LOG_DYN_LINK -lboost_log -lpthread needed for boost::log
SET(CMAKE_CXX_COMPILER /usr/bin/g++)
SET(CMAKE_C_COMPILER /usr/bin/gcc)

message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}")
message("CMAKE_CXX_FLAGS is ${CMAKE_CXX_FLAGS}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")

SET(CMAKE_MODULE_PATH ${${PROJECT_NAME}_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  tf
  std_msgs
  geometry_msgs
  sensor_msgs
  message_filters
)

# Eigen 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})

# InEKF 
# find_package(inekf REQUIRED)
# include_directories(${inekf_INCLUDE_DIRS})
# message("inekf_INCLUDE_DIR: " ${inekf_INCLUDE_DIRS})
# link_directories(${inekf_LIBRARY_DIRS})
# message("inekf_LIBRARY_DIRS: " ${inekf_LIBRARY_DIRS})

# Boost
find_package(Boost 1.58 REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
message("Boost_INCLUDE_DIR: " ${Boost_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS std_msgs sensor_msgs tf geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB inekf_core
  "${PROJECT_SOURCE_DIR}/src/core/*.cpp"
)
add_library(husky_comms "${PROJECT_SOURCE_DIR}/src/communication/husky_comms.cpp")
add_library(inekf_core ${inekf_core})
add_library(husky_measurement "${PROJECT_SOURCE_DIR}/src/utils/measurement.cpp")
add_library(utils "${PROJECT_SOURCE_DIR}/src/utils/utils.cpp")
add_library(husky_state "${PROJECT_SOURCE_DIR}/src/system/husky_state.cpp")
add_library(husky_system "${PROJECT_SOURCE_DIR}/src/system/husky_system.cpp")
add_library(body_estimator "${PROJECT_SOURCE_DIR}/src/estimator/body_estimator.cpp")
add_library(pose_publisher_node "${PROJECT_SOURCE_DIR}/src/communication/pose_publisher_node.cpp")

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(husky_estimator "${PROJECT_SOURCE_DIR}/src/husky_estimator.cpp")
target_link_libraries(husky_estimator
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    husky_comms
    inekf_core
    husky_measurement
    utils
    husky_system
    husky_state
    body_estimator
    pose_publisher_node
)


add_executable(path_publisher_node "${PROJECT_SOURCE_DIR}/src/communication/path_publisher_node.cpp")
target_link_libraries(path_publisher_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(pathodom_listener_node "${PROJECT_SOURCE_DIR}/src/communication/pathodom_listener_node.cpp")
target_link_libraries(pathodom_listener_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(gps_listener_node "${PROJECT_SOURCE_DIR}/src/communication/gps_listener_node.cpp")
target_link_libraries(gps_listener_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)