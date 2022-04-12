#ifndef _ROS_communicate_with_stm32_IncInfo_h
#define _ROS_communicate_with_stm32_IncInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace communicate_with_stm32
{

  class IncInfo : public ros::Msg
  {
    public:
      typedef ros::Time _time_type;
      _time_type time;
      float encoderData[4];

    IncInfo():
      time(),
      encoderData()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.sec);
      *(outbuffer + offset + 0) = (this->time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.nsec);
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
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
      this->time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.sec);
      this->time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.nsec);
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
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

    virtual const char * getType() override { return "communicate_with_stm32/IncInfo"; };
    virtual const char * getMD5() override { return "9084687635a8e146bce68a1506b39b6c"; };

  };

}
#endif
