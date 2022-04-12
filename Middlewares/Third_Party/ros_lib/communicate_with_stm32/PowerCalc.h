#ifndef _ROS_communicate_with_stm32_PowerCalc_h
#define _ROS_communicate_with_stm32_PowerCalc_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace communicate_with_stm32
{

  class PowerCalc : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _weight_type;
      _weight_type weight;

    PowerCalc():
      header(),
      weight(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->weight >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->weight >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->weight >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->weight >> (8 * 3)) & 0xFF;
      offset += sizeof(this->weight);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->weight =  ((uint32_t) (*(inbuffer + offset)));
      this->weight |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->weight |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->weight |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->weight);
     return offset;
    }

    virtual const char * getType() override { return "communicate_with_stm32/PowerCalc"; };
    virtual const char * getMD5() override { return "2dcd8a020e7cdec0ca30fb4b70548959"; };

  };

}
#endif
