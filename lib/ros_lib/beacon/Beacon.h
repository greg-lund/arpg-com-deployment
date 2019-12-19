#ifndef _ROS_SERVICE_Beacon_h
#define _ROS_SERVICE_Beacon_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace beacon
{

static const char BEACON[] = "beacon/Beacon";

  class BeaconRequest : public ros::Msg
  {
    public:
      typedef uint8_t _beacon_id_type;
      _beacon_id_type beacon_id;

    BeaconRequest():
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

    const char * getType(){ return BEACON; };
    const char * getMD5(){ return "ade5b780f25a324b9abc305ff59ab957"; };

  };

  class BeaconResponse : public ros::Msg
  {
    public:

    BeaconResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return BEACON; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Beacon {
    public:
    typedef BeaconRequest Request;
    typedef BeaconResponse Response;
  };

}
#endif
