#ifndef _ROS_communicate_with_stm32_Encoderinfo_h
#define _ROS_communicate_with_stm32_Encoderinfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace communicate_with_stm32
{

  class Encoderinfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      int32_t encoderData[4];

    Encoderinfo():
      header(),
      encoderData()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 4; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_encoderDatai;
      u_encoderDatai.real = this->encoderData[i];
      *(outbuffer + offset + 0) = (u_encoderDatai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoderDatai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_encoderDatai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_encoderDatai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoderData[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 4; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_encoderDatai;
      u_encoderDatai.base = 0;
      u_encoderDatai.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoderDatai.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_encoderDatai.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_encoderDatai.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->encoderData[i] = u_encoderDatai.real;
      offset += sizeof(this->encoderData[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "communicate_with_stm32/Encoderinfo"; };
    virtual const char * getMD5() override { return "3337aacb9ded5a342d6d06ff451479e8"; };

  };

}
#endif
