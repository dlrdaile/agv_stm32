#ifndef _ROS_communicate_with_stm32_MotorCmd_h
#define _ROS_communicate_with_stm32_MotorCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace communicate_with_stm32
{

  class MotorCmd : public ros::Msg
  {
    public:
      typedef const char* _cmd_type;
      _cmd_type cmd;
      typedef bool _isUrgent_type;
      _isUrgent_type isUrgent;
      uint16_t data[4];

    MotorCmd():
      cmd(""),
      isUrgent(0),
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_cmd = strlen(this->cmd);
      varToArr(outbuffer + offset, length_cmd);
      offset += 4;
      memcpy(outbuffer + offset, this->cmd, length_cmd);
      offset += length_cmd;
      union {
        bool real;
        uint8_t base;
      } u_isUrgent;
      u_isUrgent.real = this->isUrgent;
      *(outbuffer + offset + 0) = (u_isUrgent.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isUrgent);
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->data[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_cmd;
      arrToVar(length_cmd, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_cmd; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_cmd-1]=0;
      this->cmd = (char *)(inbuffer + offset-1);
      offset += length_cmd;
      union {
        bool real;
        uint8_t base;
      } u_isUrgent;
      u_isUrgent.base = 0;
      u_isUrgent.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isUrgent = u_isUrgent.real;
      offset += sizeof(this->isUrgent);
      for( uint32_t i = 0; i < 4; i++){
      this->data[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->data[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->data[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "communicate_with_stm32/MotorCmd"; };
    virtual const char * getMD5() override { return "284e19fddc4df7b8d406be945dc9b9eb"; };

  };

}
#endif
