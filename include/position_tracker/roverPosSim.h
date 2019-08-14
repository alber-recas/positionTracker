#ifndef KOBUKICONTROLLER_H
#define KOBUKICONTROLLER_H

#include <ros/ros.h>
#include <stdlib.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/GetPlan.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>


using namespace geometry_msgs;
using namespace nav_msgs;
using namespace std_msgs;

using namespace std;

namespace rover_position_simulator {
	enum class Trajectory{
		linear,
		circular
	};

	class RoverPosSim
	{
	public:
		RoverPosSim(Pose initPose, Pose goalPose, int updateRate);
		void updateActualPos();

	private:
		void updateVelLin(const Float32::ConstPtr& vel);
		void updateVelAng(const Float32::ConstPtr& vel);
		void updateRadius(const Float32::ConstPtr& rad);
		void updateInitPos(const Pose::ConstPtr& initPose);
		void updateGoalPos(const Pose::ConstPtr& goalPose);
		void updateTrajectory(const std_msgs::String::ConstPtr& trajectory);
		void linearTrajectory();
		void circularTrajectory();


		ros::NodeHandle m_nh;
		ros::Publisher m_pos_pub;
		ros::Subscriber m_velLin_sub;
		ros::Subscriber m_velAng_sub;
		ros::Subscriber m_radius_sub;
		ros::Subscriber m_init_sub;
		ros::Subscriber m_goal_sub;
		ros::Subscriber m_trajectory_sub;

		int m_updateRate;
		bool m_goalReached;
		Pose m_actual_pos;

		Pose m_init_pos;
		Pose m_goal_pos;
		Trajectory m_trajectory;
		float m_velLin;
		float m_pending;
		float m_velAng;
		float m_radius;
		double m_angle;


	};
}
#endif // RRTSTAR_H
