#ifndef _ROS_arduino_Deploy_h
#define _ROS_arduino_Deploy_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arduino
{

  class Deploy : public ros::Msg
  {
    public:
      typedef uint8_t _beacon_id_type;
      _beacon_id_type beacon_id;

    Deploy():
      beacon_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->beacon_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->beacon_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->beacon_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->beacon_id);
     return offset;
    }

    const char * getType(){ return "arduino/Deploy"; };
    const char * getMD5(){ return "ade5b780f25a324b9abc305ff59ab957"; };

  };

}
#endif