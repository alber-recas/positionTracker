#ifndef OFFBPID_H
#define OFFBPID_H

#include <ros/ros.h>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include "math.h"
#include "pid.h"

using namespace geometry_msgs;
using namespace std;

namespace offbPIDvelocity {

	class OffbPIDvelocity
	{
  public:
  OffbPIDvelocity();
	//~OffbPIDvelocity();

	void targetUpdater_cb(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void pidUpdater_cb(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void state_cb(const mavros_msgs::State::ConstPtr& msg);
	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);


  //private:

	/*
	ROS Node handler
	*/
  ros::NodeHandle m_nh;

	/*
	Publishers and Subscribers related to Drone position.
		TargetUpdater: will update the next position of the Drone.
		ActualPose: will publish periodically the actual position of the Drone.
		pidUpdate: it is a Subscriber with a callback that will upload all the PID coefficients.
	*/
	ros::Subscriber m_targetUpdater_sub;
	ros::Publisher m_actualPose_pub;
	ros::Subscriber m_pidUpdater_sub;

	/*
	MAVROS Publishers & Subscribers
	*/
	ros::Subscriber m_state_sub;
	ros::ServiceClient m_arming_client;
	ros::ServiceClient m_set_mode_client;
	ros::Publisher m_local_vel_pub;
	ros::Subscriber m_local_pos_sub;
	//ros::Publisher local_pos_pub;

	mavros_msgs::State m_current_state;
	geometry_msgs::PoseStamped m_current_pose;

	geometry_msgs::TwistStamped m_vel;
	geometry_msgs::TwistStamped m_vel_up;

  Pose m_startPose;
  Pose m_actualPose;
  Pose m_finalPose;

	PID m_pidX;
	PID m_pidY;
	PID m_pidZ;

	double m_targetPosX;
	double m_targetPosY;
	double m_targetPosZ;



  };
}
#endif