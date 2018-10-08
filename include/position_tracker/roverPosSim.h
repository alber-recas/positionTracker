#ifndef KOBUKICONTROLLER_H
#define KOBUKICONTROLLER_H

#include <ros/ros.h>
#include <stdlib.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/GetPlan.h"
#include "std_msgs/Float32.h"


using namespace geometry_msgs;
using namespace nav_msgs;
using namespace std_msgs;

using namespace std;

namespace rover_position_simulator {

	class RoverPosSim
	{
  public:
      RoverPosSim();

			void updateVel(const Float32::ConstPtr& vel);
			void updateInitPos(const Pose::ConstPtr& initPose);
			void updateGoalPos(const Pose::ConstPtr& goalPose);
			void updateActualPos();

			bool m_goalReached; //TODO review public members
			Pose m_init_pos;  	//TODO review public members
			Pose m_actual_pos;  //TODO review public members
			Pose m_goal_pos;  	//TODO review public members
			float m_vel;  			//TODO review public members
			int m_sampleRate;  	//TODO review public members

  private:
    ros::NodeHandle m_nh;

		ros::Publisher m_pos_pub;
		ros::Subscriber m_vel_sub, m_init_sub, m_goal_sub;

	};
}
#endif // RRTSTAR_H
