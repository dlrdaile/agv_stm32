/********************************************************************************
 * @author: dai le
 * @email: 965794928@qq.com
 * @date: 2022/4/11 下午1:34
 * @version: 1.0
 * @description:
 ********************************************************************************/
#ifndef YAMING_F1_HX711_H
#define YAMING_F1_HX711_H
#ifdef __cplusplus
#include "Led.h"
extern "C" {
#endif
class HX711{
public:
  HX711(GPIO_TypeDef *SCK_Port,uint16_t SCK_Pin, GPIO_TypeDef *DT_Port,uint16_t DT_Pin);
  ~HX711();
  uint32_t Get_Weight();
  void Get_Maopi();
private:
  uint32_t HX711_Read(void);

public:
  Led *HX711_SCK;
  Led *HX711_DT;
  unsigned long Weight_Maopi;
  unsigned long HX711_Buffer = 0;
  long Weight_Shiwu = 0;
  bool Flag_Error = 0;
};
#ifdef __cplusplus
}
#endif
#endif // YAMING_F1_HX711_H