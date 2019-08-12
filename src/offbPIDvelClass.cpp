/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

//#include <ros/ros.h>
#include "offbPIDvel.h"

// double r=2;
// double h=2;
// double theta;
// double wn=1;

//geometry_msgs::PoseStamped pose;


// dt -  loop interval time
// max - maximum value of manipulated variable
// min - minimum value of manipulated variable
// Kp -  proportional gain
// Kd -  derivative gain
// Ki -  Integral gain
//PID pid = PID(0.2, 3, -3, 0.3, 0, 0); //Funcional

namespace offbPIDvelocity {

OffbPIDvelocity::OffbPIDvelocity():
      m_nh("~"),
      m_pidX(0.2, 5, -5, 0.3, 0, 0),
      m_pidY(0.2, 5, -5, 0.3, 0, 0),
      m_pidZ(0.2, 5, -5, 0.3, 0, 0),
      m_targetPosX(0),m_targetPosY(0),m_targetPosZ(10)
{
  m_targetUpdater_sub = m_nh.subscribe<std_msgs::Float64MultiArray>("drone/targetPos", 1, &OffbPIDvelocity::targetUpdater_cb,this);
  m_actualPose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("drone/actualPose", 1);
  m_pidUpdater_sub = m_nh.subscribe<std_msgs::Float64MultiArray>("pid/updateCoefficients", 1, &OffbPIDvelocity::pidUpdater_cb,this);
  m_state_sub = m_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &OffbPIDvelocity::state_cb,this);
  m_local_pos_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &OffbPIDvelocity::pose_cb,this);
  //m_local_pos_pub = m_nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  m_local_vel_pub = m_nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);
  m_arming_client = m_nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  m_set_mode_client = m_nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}


void OffbPIDvelocity::targetUpdater_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    m_targetPosX = msg->data[0];
    m_targetPosY = msg->data[1];
    m_targetPosZ = msg->data[2];
}

void OffbPIDvelocity::pidUpdater_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
  //First Position of the array will choose the PID of one axis to update.
  switch((int)msg->data[0]){
    case 0:
      m_pidX.updateCoefficients(0.2,3,-3,msg->data[1],msg->data[2],msg->data[3]);
    break;
    case 1:
      m_pidY.updateCoefficients(0.2,3,-3,msg->data[1],msg->data[2],msg->data[3]);
    break;
    case 2:
      m_pidZ.updateCoefficients(0.2,3,-3,msg->data[1],msg->data[2],msg->data[3]);
    break;
    default:
      ROS_INFO("Wrong PID axis selected");
    break;
  }
  ROS_INFO("PID Updated with P:%2f D:%2f I:%2f coefficients",msg->data[1],msg->data[2],msg->data[3]);
}

void OffbPIDvelocity::state_cb(const mavros_msgs::State::ConstPtr& msg){
  //ROS_INFO("Updating drone state");
  m_current_state = *msg;
  //ROS_INFO_STREAM("Current state of FCU: "<< m_current_state.mode);
}

void OffbPIDvelocity::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    m_current_pose = *msg;
}

}

using namespace offbPIDvelocity;
int main(int argc, char **argv)
{
  int count=0;
	ROS_INFO("Entering node");
  ros::init(argc, argv, "offbPIDvelClass_node");

  OffbPIDvelocity offbPIDvelocity;
  ROS_INFO("Services suscribed node");
  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

	ROS_INFO("Connecting FCU...");
    // wait for FCU connection
    while(ros::ok() && !offbPIDvelocity.m_current_state.connected){
        ros::spinOnce();
        rate.sleep();
        if(offbPIDvelocity.m_current_state.connected==true){
          ROS_INFO_STREAM("Current state of FCU: "<< offbPIDvelocity.m_current_state.mode);
          ROS_INFO_STREAM("Connected");
        }
	      //ROS_INFO("Waiting for FCU connection...");
    }

	ROS_INFO("FCU connected");

  //  pose.pose.position.x = 0;
  //  pose.pose.position.y = 0;
  //  pose.pose.position.z = 1;

    //send a few setpoints before starting
//    for(int i = 10; ros::ok() && i > 0; --i){
//        local_pos_pub.publish(pose);
//        ros::spinOnce();
//        rate.sleep();
//    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    offbPIDvelocity.m_vel_up.twist.linear.x = 0;
    offbPIDvelocity.m_vel_up.twist.linear.y = 0;
    offbPIDvelocity.m_vel_up.twist.linear.z = 0;
    offbPIDvelocity.m_vel_up.twist.angular.x = 0;
    offbPIDvelocity.m_vel_up.twist.angular.y = 0;
    offbPIDvelocity.m_vel_up.twist.angular.z = 0;

    while(ros::ok()){

        if( offbPIDvelocity.m_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( offbPIDvelocity.m_set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !offbPIDvelocity.m_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( offbPIDvelocity.m_arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        //Update the velocities for each axis of the Drone
        offbPIDvelocity.m_vel_up.twist.linear.x = offbPIDvelocity.m_pidX.calculate(offbPIDvelocity.m_targetPosX,offbPIDvelocity.m_current_pose.pose.position.x);
        offbPIDvelocity.m_vel_up.twist.linear.y = offbPIDvelocity.m_pidY.calculate(offbPIDvelocity.m_targetPosY,offbPIDvelocity.m_current_pose.pose.position.y);
        offbPIDvelocity.m_vel_up.twist.linear.z = offbPIDvelocity.m_pidZ.calculate(offbPIDvelocity.m_targetPosZ,offbPIDvelocity.m_current_pose.pose.position.z);

        //Send through MAVROS the new vel to the drone
        offbPIDvelocity.m_local_vel_pub.publish(offbPIDvelocity.m_vel_up);

        //Just publishing the actual position of the Drone
        offbPIDvelocity.m_actualPose_pub.publish(offbPIDvelocity.m_current_pose);

        count++;
        if(count==10){
          count=0;
          // ROS_INFO("Velocity: [%.2f,%.2f,%.2f] ActualPosition: [%.3f,%.3f,%.3f] TargetPosition: [%.1f,%.1f,%.1f]",
          // offbPIDvelocity.m_vel_up.twist.linear.x,
          // offbPIDvelocity.m_vel_up.twist.linear.y,
          // offbPIDvelocity.m_vel_up.twist.linear.z,
          // offbPIDvelocity.m_current_pose.pose.position.x,
          // offbPIDvelocity.m_current_pose.pose.position.y,
          // offbPIDvelocity.m_current_pose.pose.position.z,
          // offbPIDvelocity.m_targetPosX,
          // offbPIDvelocity.m_targetPosY,
          // offbPIDvelocity.m_targetPosZ );
          ROS_INFO("ActualPosition: [%.3f,%.3f,%.3f] TargetPosition: [%.1f,%.1f,%.1f]",
          offbPIDvelocity.m_current_pose.pose.position.x,
          offbPIDvelocity.m_current_pose.pose.position.y,
          offbPIDvelocity.m_current_pose.pose.position.z,
          offbPIDvelocity.m_targetPosX,
          offbPIDvelocity.m_targetPosY,
          offbPIDvelocity.m_targetPosZ );
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
