#ifndef OFFBPID_H
#define OFFBPID_H

#include <ros/ros.h>
#include <stdlib.h>
#include "geometry_msgs/Pose.h"
#include "pid.h"

using namespace geometry_msgs;
using namespace std;

namespace offbPID {

	class OffbPID
	{
  public:
  OffbPID();

  private:

  ros::NodeHandle m_nh;

  Pose m_startPose;
  Pose m_actualPose;
  Pose m_finalPose;

	PID m_pid;

  };
}
#endif
