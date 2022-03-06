#ifndef _ROS_SERVICE_MotorControl_h
#define _ROS_SERVICE_MotorControl_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace communicate_with_stm32
{

static const char MOTORCONTROL[] = "communicate_with_stm32/MotorControl";

  class MotorControlRequest : public ros::Msg
  {
    public:
      typedef uint8_t _cmd_type;
      _cmd_type cmd;
      typedef bool _Key_type;
      _Key_type Key;
      typedef uint16_t _Period_type;
      _Period_type Period;

    MotorControlRequest():
      cmd(0),
      Key(0),
      Period(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->cmd >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cmd);
      union {
        bool real;
        uint8_t base;
      } u_Key;
      u_Key.real = this->Key;
      *(outbuffer + offset + 0) = (u_Key.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Key);
      *(outbuffer + offset + 0) = (this->Period >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Period >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Period);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->cmd =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cmd);
      union {
        bool real;
        uint8_t base;
      } u_Key;
      u_Key.base = 0;
      u_Key.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Key = u_Key.real;
      offset += sizeof(this->Key);
      this->Period =  ((uint16_t) (*(inbuffer + offset)));
      this->Period |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Period);
     return offset;
    }

    virtual const char * getType() override { return MOTORCONTROL; };
    virtual const char * getMD5() override { return "f907a92c9f3403b23c50a1670f31140d"; };

  };

  class MotorControlResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      float data[4];
      typedef ros::Time _time_type;
      _time_type time;

    MotorControlResponse():
      success(0),
      data(),
      time()
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
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_datai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
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
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_datai;
      u_datai.base = 0;
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data[i] = u_datai.real;
      offset += sizeof(this->data[i]);
      }
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
     return offset;
    }

    virtual const char * getType() override { return MOTORCONTROL; };
    virtual const char * getMD5() override { return "df955469174d3868a3ec89ab88b2fa78"; };

  };

  class MotorControl {
    public:
    typedef MotorControlRequest Request;
    typedef MotorControlResponse Response;
  };

}
#endif
