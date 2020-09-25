#ifndef __ROS_HH__
#define __ROS_HH__ 
#include "ros/ros.h"
using RosNodeUPtr   = std::unique_ptr<ros::NodeHandle>; 
using RosNodeSPtr   = boost::shared_ptr<ros::NodeHandle> ;
#endif 
