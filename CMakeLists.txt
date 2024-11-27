cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

if (POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif ()

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    roslint
    control_toolbox
    dynamic_reconfigure
    hardware_interface
    controller_interface
    forward_command_controller
    pluginlib
    message_generation
    std_msgs
    realtime_tools
    nav_msgs
    tf2_ros
    geometry_msgs
    tf2_geometry_msgs
    tf2
    tf
)

## Find system libraries
#find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
generate_dynamic_reconfigure_options(
    cfg/PID.cfg
)


catkin_package(
    INCLUDE_DIRS
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    #        ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
    CATKIN_DEPENDS
    roscpp
    controller_interface
    hardware_interface
    forward_command_controller
    pluginlib
    control_toolbox
    dynamic_reconfigure
    std_msgs
    tf2_ros
    realtime_tools
    nav_msgs
    geometry_msgs
    tf2_geometry_msgs
    tf2
    tf
    DEPENDS
    ## find_package(Eigen3) provides a non standard EIGEN3_INCLUDE_DIR instead of Eigen3_INCLUDE_DIRS.
    ## Therefore, the DEPEND does not work as expected and we need to add the directory to the INCLUDE_DIRS
    # Eigen3

    ## Boost is not part of the DEPENDS since it is only used in source files,
    ## Dependees do not depend on Boost when they depend on this package.
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    # Set manually because Eigen sets a non standard INCLUDE DIR
)

## Declare a cpp library
add_library(${PROJECT_NAME}
    src/hero_chassis_controller.cpp
)


## Declare cpp executables
#add_executable(${PROJECT_NAME}
#        src/hero_chassis_controller.cpp
#
#)
add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)
add_dependencies(hero_chassis_controller
    ${PROJECT_NAME}_gencfg)
add_executable(random_move_node
    src/random_move_node.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
target_link_libraries(random_move_node
    ${catkin_LIBRARIES}
)

#target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs

#add_dependencies(${PROJECT_NAME}
#        ${catkin_EXPORTED_TARGETS}
#)

## Specify libraries to link executable targets against
#target_link_libraries(${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
#install(
#        TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
## Mark cpp header files for installation
#install(
#        DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        FILES_MATCHING PATTERN "*.hpp"
#)
#
## Mark other files for installation
#install(
#        DIRECTORY doc
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

#if (${CATKIN_ENABLE_TESTING})
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#    ## Add gtest based cpp test target and link libraries
#    catkin_add_gtest(${PROJECT_NAME}-test
#            test/test_ros_package_template.cpp
#            test/AlgorithmTest.cpp)
#    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#endif ()

##########################
## Static code analysis ##
##########################

roslint_cpp()