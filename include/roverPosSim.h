#define ROVERPOSSIM_H
#ifndef ROVERPOSSIM_H

#include <ros/ros.h>
#include <stdlib.h>
#include "geometry_msgs/Pose.h"

using namespace geometry_msgs;
using namespace std;

namespace roverPosSim {

	class RoverPosSim
	{
  public:
  RoverPosSim();

  private:

  ros::NodeHandle m_nh;

  Pose m_startPose;
  Pose m_actualPose;
  Pose m_finalPose;


  };
}
#endif
