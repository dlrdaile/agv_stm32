#ifndef _ROS_SERVICE_InitMotor_h
#define _ROS_SERVICE_InitMotor_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "communicate_with_stm32/InitConfig.h"

namespace communicate_with_stm32
{

static const char INITMOTOR[] = "communicate_with_stm32/InitMotor";

  class InitMotorRequest : public ros::Msg
  {
    public:

    InitMotorRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return INITMOTOR; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class InitMotorResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef communicate_with_stm32::InitConfig _battery_config_type;
      _battery_config_type battery_config;
      typedef communicate_with_stm32::InitConfig _encoder_config_type;
      _encoder_config_type encoder_config;
      typedef communicate_with_stm32::InitConfig _press_config_type;
      _press_config_type press_config;

    InitMotorResponse():
      success(0),
      battery_config(),
      encoder_config(),
      press_config()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      offset += this->battery_config.serialize(outbuffer + offset);
      offset += this->encoder_config.serialize(outbuffer + offset);
      offset += this->press_config.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      offset += this->battery_config.deserialize(inbuffer + offset);
      offset += this->encoder_config.deserialize(inbuffer + offset);
      offset += this->press_config.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return INITMOTOR; };
    virtual const char * getMD5() override { return "364457133626a05be91d73e5dc6beac6"; };

  };

  class InitMotor {
    public:
    typedef InitMotorRequest Request;
    typedef InitMotorResponse Response;
  };

}
#endif
