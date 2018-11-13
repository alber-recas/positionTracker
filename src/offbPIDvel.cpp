/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "math.h"

double r=2;
double h=2;
double theta;
double count=0.0;
double wn=1;

geometry_msgs::PoseStamped pose;
geometry_msgs::TwistStamped vel;
geometry_msgs::TwistStamped vel_up;


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

void newVel(int i){

	switch (i) {
		case 0:
		vel.twist.linear.x = 3;
		vel.twist.linear.y = 0;
		vel.twist.linear.z = 1;
		break;
		case 1:
		vel.twist.linear.x = 0;
		vel.twist.linear.y = 3;
		vel.twist.linear.z = 1;
		break;
		case 2:
		vel.twist.linear.x = -3;
		vel.twist.linear.y = 0;
		vel.twist.linear.z = 1;
		break;
		case 4:
		vel.twist.linear.x = 0;
		vel.twist.linear.y = -3;
		vel.twist.linear.z = 1;
		break;
	}

}

int main(int argc, char **argv)
{
	ROS_INFO("Entering node");
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
		ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

	ROS_INFO("Services suscribed node");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);

	ROS_INFO("Connecting FCU...");
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
	ROS_INFO("Waiting for FCU connection...");
    }

	ROS_INFO("FCU connected");


    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    int poses=0;
    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    vel_up.twist.linear.x = 0;
    vel_up.twist.linear.y = 0;
    vel_up.twist.linear.z = 0;
    vel_up.twist.angular.x = 0;
    vel_up.twist.angular.y = 0;
    vel_up.twist.angular.z = 0;


    while(ros::ok()){

        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

   	   	count++;
				if(count==5)
					count=0;

				newVel(count);
        //local_pos_pub.publish(pose);

        if(current_pose.pose.position.z<10){
            vel_up.twist.linear.z += 0.1;
            local_vel_pub.publish(vel_up);
        }else{
            vel_up.twist.linear.z -= 0.1;
            local_vel_pub.publish(vel_up);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}