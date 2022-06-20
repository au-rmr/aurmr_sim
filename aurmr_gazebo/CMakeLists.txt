cmake_minimum_required(VERSION 3.0.2)
project(aurmr_gazebo)

# Load catkin and all dependencies required for this package

find_package(catkin REQUIRED COMPONENTS
        gazebo_dev
        roscpp
        image_transport
        camera_info_manager
        sensor_msgs
)

find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


catkin_package(
        CATKIN_DEPENDS
        roscpp
        image_transport
        camera_info_manager
        sensor_msgs)

install(DIRECTORY config include launch models robots urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


include_directories(
        include)
include_directories(
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(
        realsense_l515_gazebo_plugin
        src/RealSenseL515Plugin.cpp
        src/gazebo_ros_realsense_l515.cpp
)

target_link_libraries(realsense_l515_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(realsense_l515_gazebo_plugin ${catkin_EXPORTED_TARGETS})

install(
        TARGETS
        realsense_l515_gazebo_plugin
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)




