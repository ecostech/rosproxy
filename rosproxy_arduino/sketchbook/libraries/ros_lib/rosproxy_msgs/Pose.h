#ifndef _ROS_rosproxy_msgs_Pose_h
#define _ROS_rosproxy_msgs_Pose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rosproxy_msgs/Point.h"
#include "rosproxy_msgs/Quaternion.h"

namespace rosproxy_msgs
{

  class Pose : public ros::Msg
  {
    public:
      rosproxy_msgs::Point position;
      rosproxy_msgs::Quaternion orientation;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->position.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "rosproxy_msgs/Pose"; };
    const char * getMD5(){ return "1ec5a94781ee76f5aa190025baf9a1a4"; };

  };

}
#endif