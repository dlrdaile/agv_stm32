/********************************************************************************
 * @author: dai le
 * @email: 965794928@qq.com
 * @date: 2022/4/11 下午1:34
 * @version: 1.0
 * @description:
 ********************************************************************************/
#include "HX711.h"
#include "dwt_stm32_delay.h"
#include "cmsis_os.h"
#include "ros.h"
#define DELAY_US 2
extern ros::NodeHandle nh;

HX711::HX711(GPIO_TypeDef *SCK_Port, uint16_t SCK_Pin, GPIO_TypeDef *DT_Port,
             uint16_t DT_Pin) {
  HX711_SCK = new Led(SCK_Port, SCK_Pin);
  HX711_DT = new Led(DT_Port, DT_Pin);
  this->Get_Maopi();
}
HX711::~HX711() {
  if (this->HX711_SCK != NULL) {
    delete HX711_SCK;
    HX711_SCK = NULL;
  }
  if (this->HX711_DT != NULL) {
    delete HX711_DT;
    HX711_DT = NULL;
  }
}
void HX711::Get_Maopi() { this->Weight_Maopi = this->HX711_Read(); }
uint32_t HX711::Get_Weight() {
  HX711_Buffer = HX711_Read();
  if (HX711_Buffer >= Weight_Maopi) {
    Weight_Shiwu = HX711_Buffer;
    Weight_Shiwu = Weight_Shiwu - this->Weight_Maopi; //获取实物的AD采样数值。

    Weight_Shiwu = (unsigned long)((float)Weight_Shiwu / 420 + 0.05);
    //计算实物的实际重量
    //因为不同的传感器特性曲线不一样，因此，每一个传感器需要矫正这里的40这个除数。
    //当发现测试出来的重量偏大时，增加该数值。
    //如果测试出来的重量偏小时，减小改数值。
    //该数值一般在30-50之间。因传感器不同而定。
    //+0.05是为了四舍五入百分位
    if (Weight_Shiwu > 5000) {
      Flag_Error = 1;
      nh.logwarn("the weight is more than 5kg!");
    } else {
      Flag_Error = 0;
    }
  } else if (HX711_Buffer > Weight_Maopi - 3000) {
    Flag_Error = 0;
    Weight_Shiwu = 0;
  } else {
    Flag_Error = 1;
    nh.logwarn("get a negative press!");
  }
  return Weight_Shiwu;
}
uint32_t HX711::HX711_Read() {
  uint32_t count;
  uint8_t i;

  HX711_DT->On();
  DWT_Delay_us(DELAY_US);

  HX711_SCK->Off();
  DWT_Delay_us(DELAY_US);

  count = 0;
  uint32_t tick = xTaskGetTickCount();
  while (HX711_DT->Read_State()){
    vTaskDelay(1);
    if(xTaskGetTickCount() - tick > 150)
    {
      return 0;
    }
  }
  vPortEnterCritical();
  for (i = 0; i < 24; i++) {
    HX711_SCK->On();
    DWT_Delay_us(DELAY_US);
    count = count << 1;
    HX711_SCK->Off();
    DWT_Delay_us(DELAY_US);
    if (this->HX711_DT->Read_State())
      count++;
  }
  HX711_SCK->On();
  DWT_Delay_us(DELAY_US);
  HX711_SCK->Off();
  DWT_Delay_us(DELAY_US);
  vPortExitCritical();
  count ^= 0x800000;
  return count;
}