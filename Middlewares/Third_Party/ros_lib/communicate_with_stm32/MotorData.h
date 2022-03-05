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
      int8_t oneMs_encoder[4];
      int32_t encoderData[4];
      int16_t setted_speed[4];
      float fact_speed[4];

    MotorData():
      battery(0),
      oneMs_encoder(),
      encoderData(),
      setted_speed(),
      fact_speed()
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
      union {
        int8_t real;
        uint8_t base;
      } u_oneMs_encoderi;
      u_oneMs_encoderi.real = this->oneMs_encoder[i];
      *(outbuffer + offset + 0) = (u_oneMs_encoderi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->oneMs_encoder[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_encoderDatai;
      u_encoderDatai.real = this->encoderData[i];
      *(outbuffer + offset + 0) = (u_encoderDatai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoderDatai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_encoderDatai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_encoderDatai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoderData[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_setted_speedi;
      u_setted_speedi.real = this->setted_speed[i];
      *(outbuffer + offset + 0) = (u_setted_speedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_setted_speedi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->setted_speed[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_fact_speedi;
      u_fact_speedi.real = this->fact_speed[i];
      *(outbuffer + offset + 0) = (u_fact_speedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fact_speedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fact_speedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fact_speedi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fact_speed[i]);
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
      union {
        int8_t real;
        uint8_t base;
      } u_oneMs_encoderi;
      u_oneMs_encoderi.base = 0;
      u_oneMs_encoderi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->oneMs_encoder[i] = u_oneMs_encoderi.real;
      offset += sizeof(this->oneMs_encoder[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_encoderDatai;
      u_encoderDatai.base = 0;
      u_encoderDatai.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoderDatai.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_encoderDatai.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_encoderDatai.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->encoderData[i] = u_encoderDatai.real;
      offset += sizeof(this->encoderData[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_setted_speedi;
      u_setted_speedi.base = 0;
      u_setted_speedi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_setted_speedi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->setted_speed[i] = u_setted_speedi.real;
      offset += sizeof(this->setted_speed[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_fact_speedi;
      u_fact_speedi.base = 0;
      u_fact_speedi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fact_speedi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fact_speedi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fact_speedi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fact_speed[i] = u_fact_speedi.real;
      offset += sizeof(this->fact_speed[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "communicate_with_stm32/MotorData"; };
    virtual const char * getMD5() override { return "cb3b6fc6a89cf0f6a37ec6414c436f1f"; };

  };

}
#endif
