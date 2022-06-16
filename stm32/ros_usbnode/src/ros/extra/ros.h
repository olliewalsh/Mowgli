#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "STM32Hardware.h"

namespace ros
{
  typedef NodeHandle_<STM32Hardware, 25, 25, 512, 1024> NodeHandle;
}

#endif
