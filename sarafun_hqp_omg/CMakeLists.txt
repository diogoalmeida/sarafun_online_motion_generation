cmake_minimum_required(VERSION 2.8.3)
project(sarafun_hqp_omg)

#--------------------------------------------------------
# Add this to QtCreators arguments when running cmake
#	-DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel
#--------------------------------------------------------


# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    kdl_parser
    message_generation
    actionlib
    actionlib_msgs
    genmsg
    geometry_msgs
	kdl_conversions
)
find_package(Boost REQUIRED COMPONENTS system)

# Just to make included headers from msg,srv.. project name irrelevant
ADD_DEFINITIONS(-DPROJECT_NAME=${PROJECT_NAME})


message(STATUS "Eigen3 found at:" ${EIGEN3_INCLUDE_DIR})
################################################a
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)file:///home/dimpapag/sarafun/ros_ws/src/sarafun_hqp_omg/sarafun_hqp_omg/src/hqp_wrapper.cpp
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)


add_message_files(
  FILES
  obstacle.msg
)

add_action_files(
  DIRECTORY action
  FILES OnlineMotion.action
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  initHQP.srv
  addObs.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
### DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS  roscpp actionlib_msgs std_msgs message_runtime kdl_conversions
  DEPENDS orocos_kdl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        include
        ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${orocos_kdl_INCLUDE_DIRS}
)


# Set your files and resources here
set( Srcs src/main.cpp src/hqp_wrapper.cpp src/obsav_controller.cpp  src/obstacle.cpp srv/initHQP.srv srv/addObs.srv msg/obstacle.msg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


file(GLOB_RECURSE ALL_HEADERS include/*.h)

# USED only for some QTcreator stupidity between versions... ??
add_library(${PROJECT_NAME}_testlib ${Srcs} ${ALL_HEADERS})

add_executable(${PROJECT_NAME}      ${Srcs} ${ALL_HEADERS})
#set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  armadillo soth)



# Find the absolute path to the orocos-kdl library

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#!!!!!!!!!!!!!!!!!!!! in order for message gen to work from cmake/QtCreator
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp ${UI_Srcs})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_testlib ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_atlantas.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
MESSAGE( STATUS "CMAKE_BINARY_DIR:         " ${CMAKE_BINARY_DIR} )
MESSAGE( STATUS "CMAKE_CURRENT_SOURCE_DIR:         " ${CMAKE_CURRENT_SOURCE_DIR} )
MESSAGE( STATUS "catkin_INCLUDE_DIRS:         " ${catkin_INCLUDE_DIRS} )
