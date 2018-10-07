#include "position_tracker/roverPosSim.h"
//#include <geometry_msgs/PoseStamped.h>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/SetMode.h>
//#include <mavros_msgs/State.h>
//#include "math.h"

namespace rover_position_simulator{

RoverPosSim::RoverPosSim():m_nh("~"){

//  poseUpdater_sub = nh_.subscribe("/pose", 1, &KobukiController::poseUpdaterCallback, this);
	m_pos_pub = m_nh.advertise<geometry_msgs::Pose>( "/actual_pos", 10 );
	m_vel_sub = m_nh.subscribe("/vel", 1, &RoverPosSim::updateVel, this);
	m_init_sub = m_nh.subscribe("/init_pos", 1, &RoverPosSim::updateInitPos, this);
	m_goal_sub = m_nh.subscribe("/goal_pos", 1, &RoverPosSim::updateGoalPos, this);

}

RoverPosSim::updateVel(const Float32::ConstPtr& vel){
	m_vel = vel->data;
}

RoverPosSim::updateInitPos(const Pose::ConstPtr& initPose){
	m_init_pos = *initPose;
}

RoverPosSim::updateGoalPos(const Pose::ConstPtr& goalPose){
	m_goal_pos = *goalPose;
}

RoverPosSim::updateActualPos(){

	Pose actualPose = m_actual_pos;
	Pose new_pos;
	float step = vel/m_sampleRate;

	if(m_actual_pos.position.y == m_goal_pos.position.y){
		m_goalReached = true;
	}

	if(m_actual_pos.position.y == m_init_pos.position.y){
		m_goalReached = false;
	}

	if(!m_goalReached){
		//new_pos.position.x = m_actual_pos.position.x + step;
		new_pos.position.y = m_actual_pos.position.y + step;
	}else{
		new_pos.position.y = m_actual_pos.position.y - step;
	}
	// Pose distance;
	// distance.position.x = abs(m_init_pos.position.x - m_goal_pos.position.x);
	// distance.position.y = abs(m_init_pos.position.y - m_goal_pos.position.y);
	// distance.position.z = abs(m_init_pos.position.z - m_goal_pos.position.z);

	m_pos_pub.publish(new_pos);
	m_actual_pos = new_pos;
}

}

using namespace rover_position_simulator;
int main(int argc, char **argv)
{
		ROS_INFO("Entering node");
    ros::init(argc, argv, "position_tracker_node");

    ros::NodeHandle nh;

		RoverPosSim roverPosSim;
		m_sampleRate=20;
		ros::Rate loop_rate(m_sampleRate);

		//Default initial values for position and velocity
		m_vel = 1.0; // m/s
		m_init_pos.position.x = 5;
		m_init_pos.position.y = 0;
		m_init_pos.position.z = 0;

		m_goal_pos.position.x = 5;
		m_goal_pos.position.y = 5;
		m_goal_pos.position.z = 0;

		m_goalReached=false;
		m_actual_pos = m_init_pos;

    while (ros::ok()){
			//roverPosSim.updateActualPos();
  		ros::spinOnce();
  		loop_rate.sleep();
  	}

    return 0;
}
