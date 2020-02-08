#cd ~/src/Firmware
#make posix_sitl_default gazebo
#source ~/catkin_ws/devel/setup.bash    // (optional)
source ~/src/Firmware/Tools/setup_gazebo.bash ~/src/Firmware ~/src/Firmware/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/Firmware/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch
