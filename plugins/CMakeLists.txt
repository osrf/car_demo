include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(PriusHybridPlugin SHARED PriusHybridPlugin.cc PriusLogger.cc)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES})
install(TARGETS PriusHybridPlugin DESTINATION ${PRIUSCUP_PLUGIN_INSTALL_DIR})

find_package(ignition-transport3 REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

find_package(ignition-common0 REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-COMMON_CXX_FLAGS}")
include_directories(${IGNITION-COMMON_INCLUDE_DIRS})
link_directories(${IGNITION-COMMON_LIBRARY_DIRS})

add_library(MaxTimeToLivePlugin SHARED MaxTimeToLive.cc)
target_link_libraries(MaxTimeToLivePlugin gazebo ignition-common0)
install(TARGETS MaxTimeToLivePlugin DESTINATION ${PRIUSCUP_PLUGIN_INSTALL_DIR})
