#include "position_tracker/roverPosSim.h"

namespace rover_position_simulator {

	RoverPosSim::RoverPosSim(Pose initPose, Pose goalPose, int updateRate):
	m_nh("~"),
	m_updateRate(updateRate),
	m_goalReached(false),
	m_actual_pos(initPose),
	m_init_pos(initPose),
	m_goal_pos(goalPose),
	m_trajectory(Trajectory::linear),
	m_velLin(0.5),
	m_velAng(0.2),
	m_radius(10),
	m_angle(0)
	{
		m_pos_pub = m_nh.advertise<geometry_msgs::Pose>( "/offbPIDvelClass/drone/targetPos", 10 );
		//m_pos_pub = m_nh.advertise<geometry_msgs::Pose>( "/offbPIDvelClass_node/drone/targetPos", 10 );
		m_velLin_sub = m_nh.subscribe("velLin", 1, &RoverPosSim::updateVelLin, this);
		m_velAng_sub = m_nh.subscribe("velAng", 1, &RoverPosSim::updateVelAng, this);
		m_init_sub = m_nh.subscribe("initialPos", 1, &RoverPosSim::updateInitPos, this);
		m_goal_sub = m_nh.subscribe("goalPos", 1, &RoverPosSim::updateGoalPos, this);
		m_radius_sub = m_nh.subscribe("radius", 1, &RoverPosSim::updateRadius, this);
		m_trajectory_sub = m_nh.subscribe("trajectoryType", 1, &RoverPosSim::updateTrajectory, this);
		m_pending = abs(m_goal_pos.position.y - m_init_pos.position.y)/
		abs(m_goal_pos.position.x - m_init_pos.position.x);
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
			m_radius = rad->data;
		}
	}

	void RoverPosSim::updateInitPos(const Pose::ConstPtr& initPose){
		m_init_pos = *initPose;
		m_pending = abs(m_goal_pos.position.y - m_init_pos.position.y)/
								abs(m_goal_pos.position.x - m_init_pos.position.x);
		ROS_INFO("Updated goal position new pending: %f",m_pending);
	}

	void RoverPosSim::updateGoalPos(const Pose::ConstPtr& goalPose){
		m_goal_pos = *goalPose;
		m_pending = abs(m_goal_pos.position.y - m_init_pos.position.y)/
								abs(m_goal_pos.position.x - m_init_pos.position.x);
		ROS_INFO("Updated goal position new pending: %f",m_pending);
	}

	void RoverPosSim::updateTrajectory(const std_msgs::String::ConstPtr& trajectory){
		if(trajectory->data == "linear" || trajectory->data == "Linear"){
			m_trajectory = Trajectory::linear;
			ROS_INFO("Selected Linear trajectory.");
		}else if(trajectory->data == "circular" || trajectory->data == "Circular"){
			m_trajectory = Trajectory::circular;
			ROS_INFO("Selected circular trajectory.");
		}else{
			ROS_INFO("Wrong trajectory type selected.");
		}
	}

	void RoverPosSim::linearTrajectory(){

		float step = m_velLin/m_updateRate;

		if(m_pending<1){
			if(m_actual_pos.position.y >= m_goal_pos.position.y){
				m_goalReached = true;
				m_actual_pos = m_goal_pos;
			}
			if(m_actual_pos.position.y <= m_init_pos.position.y){
				m_goalReached = false;
				m_actual_pos = m_init_pos;
			}
		}else{
			if(m_actual_pos.position.x >= m_goal_pos.position.x){
				m_goalReached = true;
				m_actual_pos = m_goal_pos;
			}
			if(m_actual_pos.position.x <= m_init_pos.position.x){
				m_goalReached = false;
				m_actual_pos = m_init_pos;
			}
		}

		if(!m_goalReached){
			if(m_pending < 1.0){
				m_actual_pos.position.x += step;
				m_actual_pos.position.y = m_actual_pos.position.x*m_pending;
			}else{
				m_actual_pos.position.y += step;
				m_actual_pos.position.x = m_actual_pos.position.y/m_pending;
			}
		}else{
			if(m_pending < 1.0){
				m_actual_pos.position.x -= step;
				m_actual_pos.position.y = m_actual_pos.position.x*m_pending;
			}else{
				m_actual_pos.position.y -= step;
				m_actual_pos.position.x = m_actual_pos.position.y/m_pending;
			}
		}
	}

	void RoverPosSim::circularTrajectory(){

		float delta_time = m_velAng/m_updateRate;
		m_angle += m_velAng * delta_time;
		// x = center_x + radius * cos(m_angle);
		// y = center_y + radius * sin(m_angle);
		m_actual_pos.position.x = m_radius * cos(m_angle);
		m_actual_pos.position.y = m_radius * sin(m_angle);
		ROS_INFO_STREAM("Position [x,y]: " << m_actual_pos.position.x <<" , "<< m_actual_pos.position.y );
		ROS_INFO_STREAM("m_angle: "<< m_angle <<" Radius: "<<sqrt(pow(m_actual_pos.position.x,2)+pow(m_actual_pos.position.y,2)));
	}

	void RoverPosSim::updateActualPos(){
		switch(m_trajectory){
			case Trajectory::linear:
				linearTrajectory();
			break;
			case Trajectory::circular:
				circularTrajectory();
			break;
			default:
				linearTrajectory();
			break;
		}

		ROS_INFO_STREAM("Position [x,y]: " << m_actual_pos.position.x <<" , "<< m_actual_pos.position.y );

		m_pos_pub.publish(m_actual_pos);
	}

}

using namespace rover_position_simulator;
int main(int argc, char **argv)
{
	ROS_INFO("Entering roverPositionSimulator node");
	ros::init(argc, argv, "roverPositionSimulator");

	//Default initial values for position and velocity
	Pose initPose;
	initPose.position.x = 0;
	initPose.position.y = 0;
	initPose.position.z = 5;
	Pose goalPose;
	goalPose.position.x = 20;
	goalPose.position.y = 20;
	goalPose.position.z = 5;

	int updateRate=1;
	RoverPosSim roverPosSim(initPose,goalPose,updateRate);
	ros::Rate rate(updateRate);

	while (ros::ok()){
		ros::spinOnce();
		rate.sleep();
		roverPosSim.updateActualPos();
	}

	return 0;

}
