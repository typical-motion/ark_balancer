#ifndef __H_DISPATCH
#define __H_DISPATCH
#include <ros/ros.h>
#include "protocol.h"

void dispatch_setup(ros::NodeHandle);

void dispatch(Frame);
#endif