cmake_minimum_required(VERSION 2.8.3)
project(aurmr_gazebo)

find_package(catkin REQUIRED COMPONENTS
        gazebo_dev
        roscpp
        std_msgs
        control_toolbox
        controller_manager
        hardware_interface
        transmission_interface
        pluginlib
        joint_limits_interface
        urdf
        angles
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        controller_manager
        control_toolbox
        pluginlib
        hardware_interface
        transmission_interface
        joint_limits_interface
        urdf
        angles
        hardware_interface)

install(DIRECTORY launch worlds robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)






