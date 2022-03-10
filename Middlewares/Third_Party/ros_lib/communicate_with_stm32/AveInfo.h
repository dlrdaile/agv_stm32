#ifndef _ROS_communicate_with_stm32_AveInfo_h
#define _ROS_communicate_with_stm32_AveInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace communicate_with_stm32
{

  class AveInfo : public ros::Msg
  {
    public:
      typedef ros::Time _time_type;
      _time_type time;
      typedef uint32_t _duration_type;
      _duration_type duration;
      int32_t encoderData[4];

    AveInfo():
      time(),
      duration(0),
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
      *(outbuffer + offset + 0) = (this->duration >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->duration >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->duration >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->duration >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duration);
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
      this->duration =  ((uint32_t) (*(inbuffer + offset)));
      this->duration |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->duration |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->duration |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->duration);
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

    virtual const char * getType() override { return "communicate_with_stm32/AveInfo"; };
    virtual const char * getMD5() override { return "fef3c44d5fe108b077f4d65a5568b45e"; };

  };

}
#endif
