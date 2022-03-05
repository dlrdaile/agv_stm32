#ifndef _ROS_communicate_with_stm32_MotorData_h
#define _ROS_communicate_with_stm32_MotorData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace communicate_with_stm32
{

  class MotorData : public ros::Msg
  {
    public:
      typedef uint32_t _battery_type;
      _battery_type battery;
      uint32_t encoder[4];
      uint16_t setted_speed[4];

    MotorData():
      battery(0),
      encoder(),
      setted_speed()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->battery >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->battery >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->battery >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->battery >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery);
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->encoder[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->encoder[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->encoder[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->encoder[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoder[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->setted_speed[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->setted_speed[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->setted_speed[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->battery =  ((uint32_t) (*(inbuffer + offset)));
      this->battery |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->battery |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->battery |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->battery);
      for( uint32_t i = 0; i < 4; i++){
      this->encoder[i] =  ((uint32_t) (*(inbuffer + offset)));
      this->encoder[i] |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoder[i] |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->encoder[i] |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->encoder[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      this->setted_speed[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->setted_speed[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->setted_speed[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "communicate_with_stm32/MotorData"; };
    virtual const char * getMD5() override { return "bb1a984d6e60e05f7dae846ff6658b5e"; };

  };

}
#endif
