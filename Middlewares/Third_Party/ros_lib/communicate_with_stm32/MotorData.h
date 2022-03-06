#ifndef _ROS_communicate_with_stm32_MotorData_h
#define _ROS_communicate_with_stm32_MotorData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace communicate_with_stm32
{

  class MotorData : public ros::Msg
  {
    public:
      typedef ros::Time _time_type;
      _time_type time;
      typedef uint32_t _battery_type;
      _battery_type battery;
      float AveSpeed[4];
      float IncSpeed[4];

    MotorData():
      time(),
      battery(0),
      AveSpeed(),
      IncSpeed()
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
      *(outbuffer + offset + 0) = (this->battery >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->battery >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->battery >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->battery >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery);
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_AveSpeedi;
      u_AveSpeedi.real = this->AveSpeed[i];
      *(outbuffer + offset + 0) = (u_AveSpeedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AveSpeedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_AveSpeedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_AveSpeedi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->AveSpeed[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_IncSpeedi;
      u_IncSpeedi.real = this->IncSpeed[i];
      *(outbuffer + offset + 0) = (u_IncSpeedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_IncSpeedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_IncSpeedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_IncSpeedi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->IncSpeed[i]);
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
      this->battery =  ((uint32_t) (*(inbuffer + offset)));
      this->battery |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->battery |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->battery |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->battery);
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_AveSpeedi;
      u_AveSpeedi.base = 0;
      u_AveSpeedi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AveSpeedi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_AveSpeedi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_AveSpeedi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->AveSpeed[i] = u_AveSpeedi.real;
      offset += sizeof(this->AveSpeed[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_IncSpeedi;
      u_IncSpeedi.base = 0;
      u_IncSpeedi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_IncSpeedi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_IncSpeedi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_IncSpeedi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->IncSpeed[i] = u_IncSpeedi.real;
      offset += sizeof(this->IncSpeed[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "communicate_with_stm32/MotorData"; };
    virtual const char * getMD5() override { return "e4fa51a86f15a94fcd3290b25754df80"; };

  };

}
#endif
