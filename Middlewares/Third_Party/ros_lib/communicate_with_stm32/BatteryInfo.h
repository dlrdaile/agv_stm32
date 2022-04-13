#ifndef _ROS_communicate_with_stm32_BatteryInfo_h
#define _ROS_communicate_with_stm32_BatteryInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace communicate_with_stm32
{

  class BatteryInfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _mVoltage_type;
      _mVoltage_type mVoltage;

    BatteryInfo():
      header(),
      mVoltage(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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
      offset += this->header.deserialize(inbuffer + offset);
      this->mVoltage =  ((uint32_t) (*(inbuffer + offset)));
      this->mVoltage |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mVoltage |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->mVoltage |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->mVoltage);
     return offset;
    }

    virtual const char * getType() override { return "communicate_with_stm32/BatteryInfo"; };
    virtual const char * getMD5() override { return "8a9b3d2dcd89836905750459677d6d96"; };

  };

}
#endif
