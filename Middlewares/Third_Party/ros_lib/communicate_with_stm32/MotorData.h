#ifndef _ROS_communicate_with_stm32_MotorData_h
#define _ROS_communicate_with_stm32_MotorData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "communicate_with_stm32/AveInfo.h"
#include "communicate_with_stm32/BatteryInfo.h"

namespace communicate_with_stm32
{

  class MotorData : public ros::Msg
  {
    public:
      typedef communicate_with_stm32::AveInfo _aveInfo_type;
      _aveInfo_type aveInfo;
      float set_speed[4];
      typedef communicate_with_stm32::BatteryInfo _batteryInfo_type;
      _batteryInfo_type batteryInfo;

    MotorData():
      aveInfo(),
      set_speed(),
      batteryInfo()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->aveInfo.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_set_speedi;
      u_set_speedi.real = this->set_speed[i];
      *(outbuffer + offset + 0) = (u_set_speedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_speedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_speedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_speedi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_speed[i]);
      }
      offset += this->batteryInfo.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->aveInfo.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_set_speedi;
      u_set_speedi.base = 0;
      u_set_speedi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_speedi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_speedi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_speedi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_speed[i] = u_set_speedi.real;
      offset += sizeof(this->set_speed[i]);
      }
      offset += this->batteryInfo.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "communicate_with_stm32/MotorData"; };
    virtual const char * getMD5() override { return "71530857b3c5555f697328a1804bc00d"; };

  };

}
#endif
