#define ROVERPOSSIM_H
#ifndef ROVERPOSSIM_H

#include <ros/ros.h>
#include <stdlib.h>
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float32.h"

using namespace geometry_msgs;
using namespace std;

namespace rover_position_simulator{

	class RoverPosSim
	{
  public:
  	RoverPosSim();

  private:
		ros::NodeHandle m_nh;

		ros::Publisher m_pos_pub;
		ros::Subscriber m_vel_sub, m_init_sub, m_goal_sub;

	  Pose m_init_pos;
	  Pose m_actual_pos;
	  Pose m_goal_pos;
		float m_vel;
		bool m_goalReached;

		int m_sampleRate;

		void updateVel(const Float32::ConstPtr& vel);
		void updateInitPos(const Pose::ConstPtr& initPose);
		void updateGoalPos(const Pose::ConstPtr& goalPose);

		void updateActualPos();


  };
}
#endif
