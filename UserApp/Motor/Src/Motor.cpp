/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/24 下午7:59
* @version: 1.0
* @description: 
********************************************************************************/
#include "Motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "UserConfig.h"
#include "timers.h"

#if JLINK_DEBUG == 1
#include "SEGGER_RTT.h"
#endif

#include "ros.h"
#include "memory.h"
#include "semphr.h"
#include "HX711.h"
#include "mecumnamu.h"

extern ros::NodeHandle nh;
extern ros::Publisher EncoderPub;
extern ros::Publisher BatteryPub;
extern SemaphoreHandle_t canMutex;
extern TimerHandle_t encoderTimerHandle;
extern TimerHandle_t batteryTimerHandle;
extern HX711 hx711;
extern communicate_with_stm32::InitConfig pressCtrl;
extern TaskHandle_t PressTaskHandle;
Motor::Motor(CAN_HandleTypeDef &hcan) : timeOffSet(0, (uint32_t) (taskdelaytick * 1e6)),
                                        entimeOffSet(0, (uint32_t) (encoder_taskdelay * 1e6)) {
    this->motor2can = new Motor2Can(hcan);
    this->motor_state.encoder_info.header.frame_id = "encoder";
    this->motor_state.battery_info.header.frame_id = "battery";
}

Motor::~Motor() {
    if (this->motor2can != NULL) {
        delete this->motor2can;
        this->motor2can = NULL;
    }
}

HAL_StatusTypeDef Motor::InitState() {
    if (
            (this->motor2can->mCan->CAN_Init() != HAL_OK)
            || (this->motor2can->stop() != HAL_OK)
            || (this->topic_cmd(cmd_updateBattery) != HAL_OK)
            || (this->motor2can->clear_encoder() != HAL_OK)
            ) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef Motor::topic_cmd(const uint8_t &cmd, const int16_t *TxData) {
    HAL_StatusTypeDef cmd_result;
    string temp;
    switch (cmd) {
        case cmd_sysReset:
            cmd_result = this->motor2can->motion_system_reset();
            if (cmd_result == HAL_OK) {
                temp = "the system has been reset!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_setSpeed:
            cmd_result = this->motor2can->setSpeed(TxData[0], TxData[1], TxData[2], TxData[3]);
            if (cmd_result == HAL_OK) {
                for (int i = 0; i < 4; ++i) {
                    this->motor_state.set_speed[i] = TxData[i] * 0.0001;
                }
                temp = "set speed success!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_Stop:
            cmd_result = this->motor2can->stop();
            if (cmd_result == HAL_OK) {
                for (int i = 0; i < 4; ++i) {
                    this->motor_state.set_speed[i] = 0;
                }
                temp = "the car stop success!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_updateBattery:
            cmd_result = this->motor2can->update_battery();
            if (cmd_result == HAL_OK) {
#if JLINK_DEBUG == 1
                SEGGER_RTT_printf(0, "the receive data is %d\n", *((uint32_t *) this->CanRxBuffer));
#endif
                this->motor_state.battery_info.mVoltage = *((uint32_t *) this->motor2can->CanRxBuffer);
                this->motor_state.battery_info.header.stamp = nh.now();
                this->motor_state.battery_info.header.stamp -= this->timeOffSet;
                BatteryPub.publish(&this->motor_state.battery_info);
                temp = "the Battery value has been update!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_updateEncoderData:
            cmd_result = this->motor2can->update_encoderdata();
            if (cmd_result == HAL_OK) {
                vPortEnterCritical();
                for (int i = 0; i < 4; ++i) {
                    this->motor_state.encoder_info.encoderData[i] = ((int32_t *) this->motor2can->CanRxBuffer)[i];
                }
                this->motor_state.encoder_info.header.stamp = nh.now();
                this->motor_state.encoder_info.header.stamp -= this->entimeOffSet;
                vPortExitCritical();
                EncoderPub.publish(&this->motor_state.encoder_info);
                temp = "the Encoder data has been update!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_clearEncoder:
            cmd_result = this->motor2can->clear_encoder();
            if (cmd_result == HAL_OK) {
                temp = "the Encoder has been clear!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_directeMotion:
            static mecumnamu MW(0.134,0.135);
            cmd_result = this->motor2can->directe_motion(TxData[2], TxData[0], TxData[1]);
            if (cmd_result == HAL_OK) {
                MW.center2wheel(this->motor_state.set_speed,TxData);
                temp = "the directMotion is set successfully!";
                nh.loginfo(temp.c_str());
            }
            break;
        default:
            nh.logwarn("undefined topic cmd!");
            cmd_result = HAL_ERROR;
            break;
    }
    return cmd_result;
}

HAL_StatusTypeDef Motor::server_cmd(const communicate_with_stm32::MotorControl::Request &req,
                                    communicate_with_stm32::MotorControl::Response &res) {
    HAL_StatusTypeDef cmd_result = HAL_OK;
    switch (req.cmd) {
        case srv_startupEncoder:
            if (req.Key) {
                if (pdTRUE == xSemaphoreTake(canMutex, 1000)) {
                    cmd_result = this->motor2can->clear_encoder();
                    if (cmd_result == HAL_OK) {
                        xTimerChangePeriod(encoderTimerHandle, req.Period, 100);
                        xTimerReset(encoderTimerHandle, 100);
                        this->encoderCtrl.isOpen = true;
                        this->encoderCtrl.freq = req.Period;
                        nh.loginfo("timely Encoder has been opened!");
                    }
                    xSemaphoreGive(canMutex);
                } else {
                    cmd_result = HAL_ERROR;
                    nh.logwarn("Currently busy with work, please check later or turn on the "
                               "scheduled query function");
                }
            } else {
                xTimerStop(encoderTimerHandle, portMAX_DELAY);
                this->encoderCtrl.isOpen = false;
                nh.loginfo("timely Encoder has been closed!");
                cmd_result = HAL_OK;
            }
            break;
        case srv_startupBattery:
            if (req.Key) {
                if (xTimerChangePeriod(batteryTimerHandle, req.Period, 100) && xTimerReset(batteryTimerHandle, 100)) {
                    this->batteryCtrl.isOpen = true;
                    this->batteryCtrl.freq = req.Period;
                } else {
                    cmd_result = HAL_ERROR;
                }
            } else {
                xTimerStop(batteryTimerHandle, portMAX_DELAY);
                this->batteryCtrl.isOpen = false;
            }
            break;
        case srv_startPowerCalc:
            if (req.Key) {
                pressCtrl.isOpen = true;
                pressCtrl.freq = req.Period;
                vTaskResume(PressTaskHandle);
            } else {
                pressCtrl.isOpen = false;
                vTaskSuspend(PressTaskHandle);
            }
            break;
        case srv_setPowerZero:
            if (req.Key) {
                hx711.Get_Maopi();
            } else {
                cmd_result = HAL_ERROR;
            }
            break;
        default:
            nh.logwarn("undefined server cmd!");
            cmd_result = HAL_ERROR;
            break;
    }
    return cmd_result;
}

const char *select_name(uint8_t &cmd) {
    switch (cmd) {
        case cmd_sysReset:
            return "cmd_sysReset";
        case cmd_setSpeed:
            return "setSpeed";
        case cmd_Stop:
            return "Stop";
        case cmd_updateBattery:
            return "updateBattery";
        case cmd_updateEncoderData:
            return "updateEncoderData";
            //sever
        case srv_startPowerCalc:
            return "srv_startPowerCalc";
        case srv_startupBattery:
            return "startupBattery";
        case srv_startupEncoder:
            return "startupEncoder";
        case srv_setPowerZero:
            return "srv_setPowerZero";
        default:
            return "error";
    }
}






