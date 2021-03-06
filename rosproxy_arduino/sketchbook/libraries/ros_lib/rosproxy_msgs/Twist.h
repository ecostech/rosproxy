#ifndef _ROS_rosproxy_msgs_Twist_h
#define _ROS_rosproxy_msgs_Twist_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rosproxy_msgs/Vector3.h"

namespace rosproxy_msgs
{

  class Twist : public ros::Msg
  {
    public:
      rosproxy_msgs::Vector3 linear;
      rosproxy_msgs::Vector3 angular;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->linear.serialize(outbuffer + offset);
      offset += this->angular.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->linear.deserialize(inbuffer + offset);
      offset += this->angular.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "rosproxy_msgs/Twist"; };
    const char * getMD5(){ return "6d107193b261039abb32b01ddb75189b"; };

  };

}
#endif