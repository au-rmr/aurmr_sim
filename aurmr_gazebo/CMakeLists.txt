cmake_minimum_required(VERSION 3.0.2)
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

install(DIRECTORY config launch models robots urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)






