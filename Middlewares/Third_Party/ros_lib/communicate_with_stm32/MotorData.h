#ifndef _ROS_communicate_with_stm32_MotorData_h
#define _ROS_communicate_with_stm32_MotorData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "communicate_with_stm32/AveInfo.h"
#include "communicate_with_stm32/IncInfo.h"
#include "communicate_with_stm32/BatteryInfo.h"

namespace communicate_with_stm32
{

  class MotorData : public ros::Msg
  {
    public:
      typedef communicate_with_stm32::AveInfo _aveInfo_type;
      _aveInfo_type aveInfo;
      typedef communicate_with_stm32::IncInfo _incInfo_type;
      _incInfo_type incInfo;
      typedef communicate_with_stm32::BatteryInfo _batteryInfo_type;
      _batteryInfo_type batteryInfo;

    MotorData():
      aveInfo(),
      incInfo(),
      batteryInfo()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->aveInfo.serialize(outbuffer + offset);
      offset += this->incInfo.serialize(outbuffer + offset);
      offset += this->batteryInfo.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->aveInfo.deserialize(inbuffer + offset);
      offset += this->incInfo.deserialize(inbuffer + offset);
      offset += this->batteryInfo.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "communicate_with_stm32/MotorData"; };
    virtual const char * getMD5() override { return "e12b7b2eb4b44328aea74d774d9d7fa0"; };

  };

}
#endif
