/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/9 下午5:22
* @version: 1.0
* @description: 
********************************************************************************/
#ifndef PROJECT_CAR_COMDEVICE_H
#define PROJECT_CAR_COMDEVICE_H
#include "main.h"
#include <string>
using namespace std;
class ComDevice {
public:
    virtual void SendMessage(string data) = 0;
};


#endif //PROJECT_CAR_COMDEVICE_H