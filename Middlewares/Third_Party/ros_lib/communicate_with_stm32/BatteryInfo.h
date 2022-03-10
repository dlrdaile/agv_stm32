#ifndef _ROS_communicate_with_stm32_BatteryInfo_h
#define _ROS_communicate_with_stm32_BatteryInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace communicate_with_stm32
{

  class BatteryInfo : public ros::Msg
  {
    public:
      typedef ros::Time _time_type;
      _time_type time;
      typedef uint32_t _mVoltage_type;
      _mVoltage_type mVoltage;

    BatteryInfo():
      time(),
      mVoltage(0)
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
      *(outbuffer + offset + 0) = (this->mVoltage >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mVoltage >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mVoltage >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mVoltage >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mVoltage);
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
      this->mVoltage =  ((uint32_t) (*(inbuffer + offset)));
      this->mVoltage |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mVoltage |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->mVoltage |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->mVoltage);
     return offset;
    }

    virtual const char * getType() override { return "communicate_with_stm32/BatteryInfo"; };
    virtual const char * getMD5() override { return "f283dae6908398e343053b37b11277a8"; };

  };

}
#endif
