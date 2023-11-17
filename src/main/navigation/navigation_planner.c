/* 从任务计算机接收目标航点信息 */
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include "common/log.h"
#include "platform.h"
#include "build/build_config.h"
#include "build/debug.h"
#include "common/maths.h"
#include "io/serial.h"
#include "drivers/serial_uart.h"
#include "navigation_planner.h"

#define PLANNER_FRAME_SIZE 32

static uint8_t plannerRecieveBuffer[PLANNER_FRAME_SIZE];

/**
 *  修改了serial.h文件
 *  修改了navigation.c文件，init函数中末尾添加了plannerInit
 */
void plannerInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_PLANNER);
    if (!portConfig) {
        return false;
    }
    static serialPort_t *plannerPort = NULL;
    plannerPort = openSerialPort(SERIAL_PORT_USART3,
        FUNCTION_PLANNER, 
        plannerDataRecieve, 
        &plannerRecieveBuffer,
        115200, 
        MODE_RXTX, 
        SERIAL_NOT_INVERTED);
    UNUSED(plannerPort);
    return true;
    //plannerPort = uartOpen(USART1, NULL, NULL, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);
    //return plannerPort != NULL;
}

//以1hz的速度在调度器中更新
void plannerDataUnpack(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    //根据协议解析缓存区的数据
}

// Receive ISR callback
void plannerDataRecieve(uint16_t c, void *data)
{
    int16_t reciever;
    reciever = c;
    UNUSED(reciever);
    UNUSED(c);
    UNUSED(data);
}