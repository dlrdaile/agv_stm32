#ifndef _ROS_communicate_with_stm32_InitConfig_h
#define _ROS_communicate_with_stm32_InitConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace communicate_with_stm32
{

  class InitConfig : public ros::Msg
  {
    public:
      typedef bool _isOpen_type;
      _isOpen_type isOpen;
      typedef uint16_t _freq_type;
      _freq_type freq;

    InitConfig():
      isOpen(0),
      freq(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_isOpen;
      u_isOpen.real = this->isOpen;
      *(outbuffer + offset + 0) = (u_isOpen.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isOpen);
      *(outbuffer + offset + 0) = (this->freq >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->freq >> (8 * 1)) & 0xFF;
      offset += sizeof(this->freq);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_isOpen;
      u_isOpen.base = 0;
      u_isOpen.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isOpen = u_isOpen.real;
      offset += sizeof(this->isOpen);
      this->freq =  ((uint16_t) (*(inbuffer + offset)));
      this->freq |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->freq);
     return offset;
    }

    virtual const char * getType() override { return "communicate_with_stm32/InitConfig"; };
    virtual const char * getMD5() override { return "9d5e365a9c8ec0b2e519facbcab92ae5"; };

  };

}
#endif
