#include "position_tracker/roverPosSim.h"

namespace rover_position_simulator {

	RoverPosSim::RoverPosSim():m_nh("~"){

		m_pos_pub = m_nh.advertise<geometry_msgs::Pose>( "drone/targetPos", 10 );
		m_vel_sub = m_nh.subscribe("/vel", 1, &RoverPosSim::updateVel, this);
		m_init_sub = m_nh.subscribe("/init_pos", 1, &RoverPosSim::updateInitPos, this);
		m_goal_sub = m_nh.subscribe("/goal_pos", 1, &RoverPosSim::updateGoalPos, this);
	}

	void RoverPosSim::updateVel(const Float32::ConstPtr& vel){
		m_vel = vel->data;
	}

	void RoverPosSim::updateInitPos(const Pose::ConstPtr& initPose){
		m_init_pos = *initPose;
	}

	void RoverPosSim::updateGoalPos(const Pose::ConstPtr& goalPose){
		m_goal_pos = *goalPose;
	}

	void RoverPosSim::updateActualPos(){

		Pose actualPose = m_actual_pos;
		Pose new_pos = m_actual_pos;
		float step = m_vel/m_sampleRate;

		if(m_actual_pos.position.y >= m_goal_pos.position.y){
			m_goalReached = true;
		}

		if(m_actual_pos.position.y <= m_init_pos.position.y){
			m_goalReached = false;
		}

		if(!m_goalReached){
			//new_pos.position.x = m_actual_pos.position.x + step;
			new_pos.position.y = m_actual_pos.position.y + step;
		}else{
			new_pos.position.y = m_actual_pos.position.y - step;
		}

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
	roverPosSim.m_sampleRate=10;
	ros::Rate loop_rate(roverPosSim.m_sampleRate);

	//Default initial values for position and velocity
	roverPosSim.m_vel = 1.0; // m/s
	roverPosSim.m_init_pos.position.x = 5;
	roverPosSim.m_init_pos.position.y = 0;
	roverPosSim.m_init_pos.position.z = 0;

	roverPosSim.m_goal_pos.position.x = 5;
	roverPosSim.m_goal_pos.position.y = 5;
	roverPosSim.m_goal_pos.position.z = 0;

	roverPosSim.m_goalReached=false;
	roverPosSim.m_actual_pos = roverPosSim.m_init_pos;

	while (ros::ok()){
		roverPosSim.updateActualPos();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}
