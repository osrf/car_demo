cmake_minimum_required(VERSION 2.8.3)
project(priuscup)

set (PRIUSCUP_MAJOR_VERSION 0)
set (PRIUSCUP_MINOR_VERSION 1)
set (PRIUSCUP_PATCH_VERSION 0)
set (PRIUSCUP_VERSION_FULL ${PRIUSCUP_MAJOR_VERSION}.${PRIUSCUP_MINOR_VERSION}.${PRIUSCUP_PATCH_VERSION})

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

find_package(gazebo 8 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES})
# install (TARGETS ${src} DESTINATION ${PRIUSCUP_PLUGIN_INSTALL_DIR})


# TODO(tfoote) replace installers add_subdirectory(models)
# TODO(tfoote) replace installers add_subdirectory(worlds)
