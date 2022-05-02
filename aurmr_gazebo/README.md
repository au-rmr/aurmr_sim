# aurmr_gazebo

Simple simulation of the AURMR UR16e workcell with the Robotiq 2F-85 gripper. Includes [a hacked version of the Realsense Gazebo plugin](https://github.com/ZohebAbai/gazebo_ros_l515) written by Zoheb Abai which works with the L515's sensor configuration. 

## Usage

    roslaunch aurmr_gazebo fc.launch

Press play. Add the Motion Planning view in RViz to interactively create and execute motion plans.