/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/24 下午7:59
* @version: 1.0
* @description: 
********************************************************************************/
#include "Motor.h"
#include "command_length.h"
#include "Usart.h"
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

#define taskdelaytick 1
#define encoder_taskdelay 2
extern ros::NodeHandle nh;
extern SemaphoreHandle_t canMutex;
extern TimerHandle_t encoderTimerHandle;
extern TimerHandle_t batteryTimerHandle;
extern TimerHandle_t rosPubHandle;

static inline string chIDName(const uint32_t &ExtID) {
    string name;
    name.resize(50);
    switch (ExtID) {
        case setSpeed_msgID:
            name = "setSpeed";
            break;
        case sysReset_msgID:
            name = "sysReset";
            break;
        case mvDirection_msgID:
            name = "mvDirection";
            break;
        case battery_msgID:
            name = "battery";
            break;
        case oneMS_Encoder_msgID:
            name = "oneMS";
        case EncoderData_msgID:
            name = "encoder";
    }
    name += " is error because: ";
    return name;
}

HAL_StatusTypeDef Motor::verifyReceive(const CanStatusTypeDef &transtatus, const uint32_t &ExtID) {
    if (transtatus == CAN_OK) {
        CanStatusTypeDef can_receive_result = this->mCan->CAN_ReadMsg(ExtID, this->CanRxBuffer);
        if (can_receive_result == CAN_OK) {
            return HAL_OK;
        } else {
            string name = chIDName(ExtID);
            switch (can_receive_result) {
                case CAN_GET_MESSAGE_ERROR:
                    name += "CAN_GET_MESSAGE_ERROR";
                    break;
                case CAN_RECEIVE_ERROR:
                    name += "CAN_RECEIVE_ERROR";
                    break;
                case CAN_NO_RECEIVE_ERROR:
                    name += "CAN_NO_RECEIVE_ERROR";
                    break;
                default:
                    name += "no known the reason";
                    break;
            }
            nh.logwarn(name.c_str());
        }

    } else {
        string name = chIDName(ExtID);
        name += "transmit data error";
        nh.logwarn(name.c_str());
    }
    return HAL_ERROR;
}

Motor::Motor(CAN_HandleTypeDef &hcan) : timeOffSet(0, (uint32_t) (taskdelaytick * 1e6)),
                                        entimeOffSet(0, (uint32_t) (encoder_taskdelay * 1e6)) {
    this->mCan = new Can(hcan);
}

Motor::~Motor() {
    if (this->mCan != NULL) {
        delete this->mCan;
        this->mCan = NULL;
    }
}

/**
 * @brief  设置对应的参数
 * @param v1  右前轮
 * @param v2  左前轮
 * @param v3  左后轮
 * @param v4  右后轮
 * @return
 */

HAL_StatusTypeDef Motor::setSpeed(int16_t v1, int16_t v2, int16_t v3, int16_t v4) {
    v2 *= -1;
    v3 *= -1;
    int16_t speed[4] = {v1, v2, v3, v4};
    result = this->mCan->CAN_SendMsg(setSpeed_msgID, (uint8_t *) speed, DLSEND_41);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the setSpeed msgID is:0x%08x\n", setSpeed_msgID);
#endif
    HAL_Delay(taskdelaytick);
    if (this->verifyReceive(result, setSpeed_msgID) != HAL_OK) {
        return HAL_ERROR;
    }
    extern bool isInitial;
    if (isInitial) {
        vPortEnterCritical();
        memcpy(this->motor_state.setted_speed, speed, 8);
        vPortExitCritical();
    }
    return HAL_OK;
}

HAL_StatusTypeDef Motor::motion_system_reset() {
    result = this->mCan->CAN_SendMsg(sysReset_msgID, nullptr, DLSEND_25);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the reset msgID is:0x%08x\n", sysReset_msgID);
#endif
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, sysReset_msgID);
}

HAL_StatusTypeDef Motor::XY_motion(int16_t speed_x, int16_t speed_y) {
    int16_t raw_data[4] = {0, 0, speed_y, speed_x};
    result = mCan->CAN_SendMsg(mvDirection_msgID, (uint8_t *) raw_data, DLSEND_42);
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, mvDirection_msgID);
}

HAL_StatusTypeDef Motor::swerve_motion(int16_t radius, int16_t speed) {
    if (radius == 0) return HAL_ERROR;
    int16_t raw_data[4] = {0, (int16_t) (speed / radius), 0, speed};
    result = mCan->CAN_SendMsg(mvDirection_msgID, (uint8_t *) raw_data, DLSEND_42);
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, mvDirection_msgID);
}


HAL_StatusTypeDef Motor::rotate_motion(int16_t rotate_speed) {
    int16_t raw_data[4] = {0, rotate_speed, 0, 0};
    result = mCan->CAN_SendMsg(mvDirection_msgID, (uint8_t *) raw_data, DLSEND_42);
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, mvDirection_msgID);
}

HAL_StatusTypeDef Motor::update_battery() {
    result = mCan->CAN_SendMsg(battery_msgID, nullptr, DLSEND_51);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the battery msgID is:0x%08x\n", battery_msgID);
#endif
    HAL_Delay(taskdelaytick);
    if (HAL_OK == this->verifyReceive(result, battery_msgID)) {
#if JLINK_DEBUG == 1
        SEGGER_RTT_printf(0, "the receive data is %d\n", *((uint32_t *) this->CanRxBuffer));
#endif
        this->motor_state.motorData.batteryInfo.mVoltage = *((uint32_t *) this->CanRxBuffer);
        if (this->batteryCtrl.IsOpen) {
            this->motor_state.motorData.batteryInfo.time = nh.now();
            this->motor_state.motorData.batteryInfo.time -= this->timeOffSet;
        }
        return HAL_OK;
    } else
        return HAL_ERROR;
}

HAL_StatusTypeDef Motor::update_oneMs_encoder() {
    result = mCan->CAN_SendMsg(oneMS_Encoder_msgID, NULL, DLSEND_48);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the update_oneMs_encoder msgID is:0x%08x\n", oneMS_Encoder_msgID);
#endif
    HAL_Delay(encoder_taskdelay);
    if (HAL_OK == this->verifyReceive(result, oneMS_Encoder_msgID)) {
        for (int i = 0; i < 4; ++i) {
            this->motor_state.oneMs_encoder[i] = (int8_t) (((uint32_t *) this->CanRxBuffer)[i]);
        }
        this->motor_state.motorData.incInfo.time = nh.now();
        this->motor_state.motorData.incInfo.time -= this->entimeOffSet;
        return HAL_OK;
    } else
        return HAL_ERROR;
}

HAL_StatusTypeDef Motor::stop() {
    return this->setSpeed(0, 0, 0, 0);
}

HAL_StatusTypeDef Motor::InitState() {
    if (
            (this->mCan->CAN_Init() != HAL_OK)
            || (this->stop() != HAL_OK)
            || (this->update_battery() != HAL_OK)
            ) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef Motor::update_encoderdata() {
    result = mCan->CAN_SendMsg(EncoderData_msgID, NULL, DLSEND_48);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the update_encoderdata msgID is:0x%08x\n", EncoderData_msgID);
#endif
    HAL_Delay(encoder_taskdelay);
    if (HAL_OK == this->verifyReceive(result, EncoderData_msgID)) {
        vPortEnterCritical();
        this->motor_state.last_encData.check_time = this->motor_state.current_encData.check_time;
        memcpy(this->motor_state.last_encData.encoder_data, this->motor_state.current_encData.encoder_data, 16);
        this->motor_state.current_encData.check_time = HAL_GetTick() - encoder_taskdelay;
        memcpy(this->motor_state.current_encData.encoder_data, this->CanRxBuffer, 16);
        this->motor_state.motorData.aveInfo.time = nh.now();
        this->motor_state.motorData.aveInfo.time -= this->entimeOffSet;
        //TODO:考虑是否要避免溢出
       /* for (int i = 0; i < 4; ++i) {
            uint32_t out = 1 << 31;
            if(this->motor_state.current_encData.encoder_data[i] >= (out - 1) || this->motor_state.current_encData.encoder_data[i] <= -65000)
            {
                this->clear_encoder();
                break;
            }
        }*/
        vPortExitCritical();
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef Motor::clear_encoder() {
    result = this->mCan->CAN_SendMsg(ClearEncoder_msgID, nullptr, DLSEND_25);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the clear_encoder msgID is:0x%08x\n", ClearEncoder_msgID);
#endif
    HAL_Delay(taskdelaytick);
    if (HAL_OK == this->verifyReceive(result, ClearEncoder_msgID)) {
        memset(this->motor_state.current_encData.encoder_data, 0,
               sizeof(this->motor_state.current_encData.encoder_data));
        memset(this->motor_state.last_encData.encoder_data, 0,
               sizeof(this->motor_state.current_encData.encoder_data));
        this->motor_state.current_encData.check_time = HAL_GetTick() - taskdelaytick;
        this->motor_state.last_encData.check_time = HAL_GetTick() - taskdelaytick;
        return HAL_OK;
    } else
        return HAL_ERROR;
}

HAL_StatusTypeDef Motor::topic_cmd(const uint8_t &cmd, const int16_t *TxData) {
    HAL_StatusTypeDef cmd_result;
    string temp;
    switch (cmd) {
        case cmd_sysReset:
            cmd_result = this->motion_system_reset();
            if (cmd_result == HAL_OK) {
                temp = "the system has been reset!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_setSpeed:
            cmd_result = this->setSpeed(TxData[0], TxData[1], TxData[2], TxData[3]);
            if (cmd_result == HAL_OK) {
                temp = "set speed success!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_Stop:
            cmd_result = this->stop();
            if (cmd_result == HAL_OK) {
                temp = "the car stop success!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_updateBattery:
            cmd_result = this->update_battery();
            if (cmd_result == HAL_OK) {
                temp = "the Battery value has been update!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_updateEncoderData:
            cmd_result = this->update_encoderdata();
            if (cmd_result == HAL_OK) {
                temp = "the Encoder data has been update!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_getIncSpeed:
            cmd_result = this->update_oneMs_encoder();
            if (cmd_result == HAL_OK) {
                vPortEnterCritical();
                memcpy(this->motor_state.motorData.incInfo.encoderData, this->motor_state.oneMs_encoder, 4);
                vPortExitCritical();
                temp = "the IncSpeed has been update!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_getAveSpeed:
            if (this->encoderCtrl.IsOpen) {
                cmd_result = this->update_encoderdata();
                if (cmd_result == HAL_OK) {
                    vPortEnterCritical();
                    for (int i = 0; i < 4; ++i) {
                        this->motor_state.motorData.aveInfo.encoderData[i] =
                                this->motor_state.current_encData.encoder_data[i]
                                - this->motor_state.last_encData.encoder_data[i];
                    }
                    this->motor_state.motorData.aveInfo.duration = this->motor_state.current_encData.check_time -
                                                                    this->motor_state.last_encData.check_time;
                    vPortExitCritical();
                    temp = "success to get the AveSpeed!";
                    nh.loginfo(temp.c_str());
                }
            } else {
                nh.logwarn("if you want to get the speed,"
                           "please open the channel to check the encoder!");
                return HAL_ERROR;
            }
            break;
        case cmd_clearEncoder:
            cmd_result = this->clear_encoder();
            if (cmd_result == HAL_OK) {
                temp = "the Encoder has been clear!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_xyMotion:
            cmd_result = this->XY_motion(TxData[0], TxData[1]);
            if (cmd_result == HAL_OK) {
                temp = "the XYmotion is set successfully!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_swerveMotion:
            cmd_result = this->swerve_motion(TxData[0], TxData[1]);
            if (cmd_result == HAL_OK) {
                temp = "the swerveMotion is set successfully!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_rotateMotion:
            cmd_result = this->rotate_motion(TxData[0]);
            if (cmd_result == HAL_OK) {
                temp = "the rotateMotion is set successfully!";
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
        case cmd_checkAveSpeed:
            if (this->encoderCtrl.IsOpen) {
                //若1秒钟内没有得到互斥锁，则返回错误
                if (xSemaphoreTake(canMutex, 1000) == pdTRUE) {
                    cmd_result = this->update_encoderdata();
                    if (cmd_result == HAL_OK) {
                        vPortEnterCritical();
                        for (int i = 0; i < 4; ++i) {
                            this->motor_state.motorData.aveInfo.encoderData[i] =
                                    this->motor_state.current_encData.encoder_data[i]
                                    - this->motor_state.last_encData.encoder_data[i];
                        }
                        memcpy(res.data, this->motor_state.motorData.aveInfo.encoderData, 16);
                        res.time = this->motor_state.motorData.aveInfo.time;
                        this->motor_state.motorData.aveInfo.duration = this->motor_state.current_encData.check_time -
                                                                       this->motor_state.last_encData.check_time;
                        vPortExitCritical();
                    }
                    xSemaphoreGive(canMutex);
                } else {
                    nh.logwarn("Currently busy with work, please check later or turn on the "
                               "scheduled query function");
                    cmd_result = HAL_ERROR;
                }
            } else {
                cmd_result = HAL_ERROR;
                nh.logwarn("if you want to get the speed,please open the channel to check the encoder!");
            }
            break;
        case cmd_checkInsSpeed:
            if (xSemaphoreTake(canMutex, 1000) == pdTRUE) {
                cmd_result = this->update_oneMs_encoder();
                if (cmd_result == HAL_OK) {
                    vPortEnterCritical();
                    for (int i = 0; i < 4; ++i) {
                        this->motor_state.motorData.incInfo.encoderData[i] = this->motor_state.oneMs_encoder[i];
                        res.data[i] = this->motor_state.motorData.incInfo.encoderData[i];
                    }
                    res.time = this->motor_state.motorData.incInfo.time;
                    vPortExitCritical();
                }
                xSemaphoreGive(canMutex);
            } else {
                nh.logwarn("Currently busy with work, please check later or turn on the "
                           "scheduled query function");
                cmd_result = HAL_ERROR;
            }
            break;
        case cmd_checkEncoderData:
            if (xSemaphoreTake(canMutex, 1000) == pdTRUE) {
                cmd_result = this->update_encoderdata();
                if (cmd_result == HAL_OK) {
                    vPortEnterCritical();
                    memcpy(res.data, this->motor_state.current_encData.encoder_data, 16);
                    res.time = this->motor_state.motorData.aveInfo.time;
                    vPortExitCritical();
                }
                xSemaphoreGive(canMutex);
            } else {
                nh.logwarn("Currently busy with work, please check later or turn on the "
                           "scheduled query function");
                cmd_result = HAL_ERROR;
            }
            break;
        case cmd_checkOneMsEncoder:
            if (xSemaphoreTake(canMutex, 1000) == pdTRUE) {
                cmd_result = this->update_oneMs_encoder();
                if (cmd_result == HAL_OK) {
                    vPortEnterCritical();
                    for (int i = 0; i < 4; ++i) {
                        res.data[i] = this->motor_state.oneMs_encoder[i];
                    }
                    res.time = this->motor_state.motorData.incInfo.time;
                    vPortExitCritical();
                }
                xSemaphoreGive(canMutex);
            } else {
                nh.logwarn("Currently busy with work, please check later or turn on the "
                           "scheduled query function");
                cmd_result = HAL_ERROR;
            }
            break;
        case cmd_checkbattery:
            if (pdTRUE != xSemaphoreTake(canMutex, 1000)) {
                cmd_result = HAL_ERROR;
                nh.logwarn("Currently busy with work, please check later or turn on the "
                           "scheduled query function");
            } else {
                cmd_result = this->update_battery();
                if (cmd_result == HAL_OK) {
                    res.data[0] = this->motor_state.motorData.batteryInfo.mVoltage;
                    res.time = this->motor_state.motorData.batteryInfo.time;
                }
                xSemaphoreGive(canMutex);
            }
            break;
        case cmd_startupEncoder:
            if (req.Key) {
                if (pdTRUE == xSemaphoreTake(canMutex, 1000)) {
                    cmd_result = this->clear_encoder();
                    if (cmd_result == HAL_OK) {
                        xTimerChangePeriod(encoderTimerHandle, req.Period, 100);
                        xTimerReset(encoderTimerHandle, 100);
                        this->encoderCtrl.IsOpen = true;
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
                this->motor_state.motorData.aveInfo.duration = 0;
                xTimerStop(encoderTimerHandle, portMAX_DELAY);
                this->encoderCtrl.IsOpen = false;
                nh.loginfo("timely Encoder has been closed!");
                cmd_result = HAL_OK;
            }
            break;
        case cmd_startupBattery:
            if (req.Key) {
                if (xTimerChangePeriod(batteryTimerHandle, req.Period, 100) && xTimerReset(batteryTimerHandle, 100)) {
                    this->batteryCtrl.IsOpen = true;
                    this->batteryCtrl.freq = req.Period;
                } else {
                    cmd_result = HAL_ERROR;
                }
            } else {
                xTimerStop(batteryTimerHandle, portMAX_DELAY);
                this->batteryCtrl.IsOpen = false;
            }
            break;
        case cmd_controlPub:
            if (req.Key) {
                if (xTimerChangePeriod(rosPubHandle, req.Period, 100) && xTimerReset(rosPubHandle, 100)) {
                    this->rosPubCtrl.IsOpen = true;
                    this->rosPubCtrl.freq = req.Period;
                } else {
                    cmd_result = HAL_ERROR;
                }
            } else {
                if (xTimerStop(rosPubHandle, 100))
                    this->rosPubCtrl.IsOpen = false;
                else {
                    cmd_result = HAL_ERROR;
                }
            }
            break;
        case cmd_checkSettedSpeed:
            res.time = nh.now();
            for (int i = 0; i < 4; ++i) {
                res.data[i] = (float) this->motor_state.setted_speed[i];
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
        case cmd_getIncSpeed:
            return "getIncSpeed";
        case cmd_clearEncoder:
            return "clearEncoder";
        case cmd_xyMotion:
            return "xyMotion";
        case cmd_swerveMotion:
            return "swerveMotion";
        case cmd_rotateMotion:
            return "rotateMotion";
        case cmd_checkAveSpeed:
            return "checkAveSpeed";
        case cmd_checkInsSpeed:
            return "checkInsSpeed";
        case cmd_checkEncoderData:
            return "checkEncoderData";
        case cmd_checkOneMsEncoder:
            return "checkOneMsEncoder";
        case cmd_checkbattery:
            return "checkbattery";
        case cmd_startupBattery:
            return "startupBattery";
        case cmd_startupEncoder:
            return "startupEncoder";
        case cmd_getAveSpeed:
            return "getAveSpeed";
        case cmd_controlPub:
            return "controlPub";
        case cmd_checkSettedSpeed:
            return "checkSettedSpeed";
        default:
            return "error";
    }
}






