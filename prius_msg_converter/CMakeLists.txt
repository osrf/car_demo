cmake_minimum_required(VERSION 2.8.3)
project(prius_msg_converter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  prius_msgs
  geometry_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
CATKIN_DEPENDS roscpp prius_msgs geometry_msgs nav_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(prius_msg_to_cmd_vel_src
  src/to_cmd_vel.cpp
  src/to_cmd_vel_node.cpp
  )
add_executable(prius_msg_to_cmd_vel ${prius_msg_to_cmd_vel_src})
target_link_libraries(prius_msg_to_cmd_vel ${catkin_LIBRARIES})

#############
## Install ##
#############
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
