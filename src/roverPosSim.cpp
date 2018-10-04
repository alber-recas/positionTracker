/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <roverPosSim/roverPosSim.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/SetMode.h>
//#include <mavros_msgs/State.h>
//#include "math.h"


RoverPosSim::RoverPosSim():m_nh("~"){

//  poseUpdater_sub = nh_.subscribe("/pose", 1, &KobukiController::poseUpdaterCallback, this);

}

int main(int argc, char **argv)
{
	ROS_INFO("Entering node");
    ros::init(argc, argv, "position_tracker_node");
    ros::NodeHandle nh;


    while (ros::ok()){
  		// if(!kobukiController.goalReached){
  		// 	kobukiController.step();
  		// }
  		ros::spinOnce();
  		loop_rate.sleep();
  	}

    return 0;
}
