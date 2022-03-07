#ifndef _ROS_communicate_with_stm32_MotorData_h
#define _ROS_communicate_with_stm32_MotorData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "ros/duration.h"

namespace communicate_with_stm32
{

  class MotorData : public ros::Msg
  {
    public:
      typedef ros::Time _Inctime_type;
      _Inctime_type Inctime;
      typedef ros::Time _Avetime_type;
      _Avetime_type Avetime;
      typedef ros::Time _batterytime_type;
      _batterytime_type batterytime;
      typedef ros::Duration _Aveduration_type;
      _Aveduration_type Aveduration;
      typedef uint32_t _battery_type;
      _battery_type battery;
      float AveSpeed[4];
      float IncSpeed[4];

    MotorData():
      Inctime(),
      Avetime(),
      batterytime(),
      Aveduration(),
      battery(0),
      AveSpeed(),
      IncSpeed()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->Inctime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Inctime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Inctime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Inctime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Inctime.sec);
      *(outbuffer + offset + 0) = (this->Inctime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Inctime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Inctime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Inctime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Inctime.nsec);
      *(outbuffer + offset + 0) = (this->Avetime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Avetime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Avetime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Avetime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Avetime.sec);
      *(outbuffer + offset + 0) = (this->Avetime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Avetime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Avetime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Avetime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Avetime.nsec);
      *(outbuffer + offset + 0) = (this->batterytime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->batterytime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->batterytime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->batterytime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->batterytime.sec);
      *(outbuffer + offset + 0) = (this->batterytime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->batterytime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->batterytime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->batterytime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->batterytime.nsec);
      *(outbuffer + offset + 0) = (this->Aveduration.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Aveduration.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Aveduration.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Aveduration.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Aveduration.sec);
      *(outbuffer + offset + 0) = (this->Aveduration.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Aveduration.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Aveduration.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Aveduration.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Aveduration.nsec);
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
      this->Inctime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->Inctime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Inctime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Inctime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Inctime.sec);
      this->Inctime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->Inctime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Inctime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Inctime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Inctime.nsec);
      this->Avetime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->Avetime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Avetime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Avetime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Avetime.sec);
      this->Avetime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->Avetime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Avetime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Avetime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Avetime.nsec);
      this->batterytime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->batterytime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->batterytime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->batterytime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->batterytime.sec);
      this->batterytime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->batterytime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->batterytime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->batterytime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->batterytime.nsec);
      this->Aveduration.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->Aveduration.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Aveduration.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Aveduration.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Aveduration.sec);
      this->Aveduration.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->Aveduration.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Aveduration.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Aveduration.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Aveduration.nsec);
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
    virtual const char * getMD5() override { return "9b850f0d908b406546b3da1dbf478e68"; };

  };

}
#endif
