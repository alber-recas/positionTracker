#include "position_tracker/roverPosSim.h"

namespace rover_position_simulator {

	RoverPosSim::RoverPosSim():m_nh("~"){
		m_pos_pub = m_nh.advertise<std_msgs::Float64MultiArray>( "/offbPIDvelClass_node/drone/targetPos", 10 );
		//m_pos_pub = m_nh.advertise<geometry_msgs::Pose>( "/offbPIDvelClass_node/drone/targetPos", 10 );
		m_velLin_sub = m_nh.subscribe("velLin", 1, &RoverPosSim::updateVelLin, this);
		m_velAng_sub = m_nh.subscribe("velAng", 1, &RoverPosSim::updateVelAng, this);
		m_init_sub = m_nh.subscribe("init_pos", 1, &RoverPosSim::updateInitPos, this);
		m_goal_sub = m_nh.subscribe("goal_pos", 1, &RoverPosSim::updateGoalPos, this);
		m_radius_sub = m_nh.subscribe("radius_pos", 1, &RoverPosSim::updateRadius, this);
	}

	void RoverPosSim::updateVelLin(const Float32::ConstPtr& vel){
		if(vel->data > 0){
			m_velLin = vel->data;
		}
	}

	void RoverPosSim::updateVelAng(const Float32::ConstPtr& vel){
		if(vel->data > 0){
			m_velAng = vel->data;
		}
	}

	void RoverPosSim::updateRadius(const Float32::ConstPtr& rad){
		if(rad->data > 0){
			m_rad = rad->data;
		}
	}

	void RoverPosSim::updateInitPos(const Pose::ConstPtr& initPose){
		m_init_pos = *initPose;
	}

	void RoverPosSim::updateGoalPos(const Pose::ConstPtr& goalPose){
		m_goal_pos = *goalPose;
	}


	Pose RoverPosSim::linearTrajectory(){
		Pose actualPose = m_actual_pos;
		Pose new_pos = m_actual_pos;
		float step = m_velLin/m_sampleRate;

		if(m_actual_pos.position.y >= m_goal_pos.position.y){
			m_goalReached = true;
		}

		if(m_actual_pos.position.y <= m_init_pos.position.y){
			m_goalReached = false;
		}

		if(!m_goalReached){
			new_pos.position.x = m_actual_pos.position.x + step;
			new_pos.position.y = m_actual_pos.position.y + step;
		}else{
			new_pos.position.x = m_actual_pos.position.x - step;
			new_pos.position.y = m_actual_pos.position.y - step;
		}
		return new_pos;
	}


	void RoverPosSim::circularTrajectory(){

		float delta_time = m_velAng/m_sampleRate;
		angle += m_velAng * delta_time;
		// x = center_x + radius * cos(angle);
		// y = center_y + radius * sin(angle);
		m_actual_pos.position.x = m_rad * cos(angle);
		m_actual_pos.position.y = m_rad * sin(angle);
		ROS_INFO_STREAM("Position [x,y]: " << m_actual_pos.position.x <<" , "<< m_actual_pos.position.y );
		ROS_INFO_STREAM("Angle: "<< angle);
	}

	void RoverPosSim::updateActualPos(){
		// m_actual_pos = linearTrajectory();
		circularTrajectory();
		std_msgs::Float64MultiArray new_pos;
		new_pos.data.resize(3);// = (double*)malloc(sizeof(double)*3);
		new_pos.data[0]= m_actual_pos.position.x;
		new_pos.data[1]= m_actual_pos.position.y;
		new_pos.data[2]= m_init_pos.position.z;
		ROS_INFO_STREAM("Position [x,y]: " << m_actual_pos.position.x <<" , "<< m_actual_pos.position.y );

		m_pos_pub.publish(new_pos);
	}

}

using namespace rover_position_simulator;
int main(int argc, char **argv)
{
	ROS_INFO("Entering node");
	ros::init(argc, argv, "position_tracker_node");

	//ros::NodeHandle nh;

	RoverPosSim roverPosSim;
	roverPosSim.m_sampleRate=1;
	ros::Rate loop_rate(roverPosSim.m_sampleRate);

	//Default initial values for position and velocity
	roverPosSim.m_velLin = 0.1; // m/s
	roverPosSim.m_velAng = 0.2; // rad/s
	roverPosSim.m_rad = 5; // m
	roverPosSim.m_init_pos.position.x = 0;
	roverPosSim.m_init_pos.position.y = 0;
	roverPosSim.m_init_pos.position.z = 5;

	roverPosSim.m_goal_pos.position.x = 20;
	roverPosSim.m_goal_pos.position.y = 20;
	roverPosSim.m_goal_pos.position.z = 5;

	roverPosSim.m_goalReached=false;
	roverPosSim.m_actual_pos = roverPosSim.m_init_pos;

	while (ros::ok()){
		roverPosSim.updateActualPos();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}
