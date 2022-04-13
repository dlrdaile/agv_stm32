/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/4/12 下午9:30
* @version: 1.0
* @description: 
********************************************************************************/
#ifndef PROJECT_CAR_FREERTOS_TASK_H
#define PROJECT_CAR_FREERTOS_TASK_H
#ifdef __cplusplus
extern "C" {
#endif
enum {
    rosEvent = 1,
    canNormalEvent = 1 << 1,
    canUrgentEvent = 1 << 2
};
#ifdef __cplusplus
}
#endif
#endif //PROJECT_CAR_FREERTOS_TASK_H