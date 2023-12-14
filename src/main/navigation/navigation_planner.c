/* 从任务计算机接收目标航点信息 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "common/log.h"
#include "platform.h"
#include "build/build_config.h"
#include "build/debug.h"
#include "common/maths.h"
#include "io/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/time.h"
#include "fc/rc_modes.h"
#include "fc/rc_controls.h"
#include "navigation_planner.h"

#define PLANNER_FRAME_SIZE 12
#define FRAME_START_1 0xFF
#define FRAME_START_2 0xAA
#define FRAME_END_1 0xDD
#define FRAME_END_2 0xEE

typedef enum {
    WAIT_START = 0,
    START_STANDBY,
    IN_FRAME,
    END_STANDBY,
} PlannerUsartReceiveState_e;

static PlannerUsartReceiveState_e state = WAIT_START;
static uint8_t plannerRecieveBuffer[PLANNER_FRAME_SIZE];
static uint8_t bufferIndex = 0;
static timeMs_t latestRecieveTime = 0;
static uint16_t plannerCommand[3];
static bool plannerCommandValidity = false;

bool isNumInRange(uint16_t num, uint16_t min, uint16_t max)
{
    bool numInRange = ((num >= min)&&(num <= max)) ? true : false;
    return numInRange;
}

/**
 *  修改了serial.h文件
 *  修改了navigation.c文件，init函数中末尾添加了plannerInit
 */
void plannerInit(void)
{
    static serialPort_t *plannerPort = NULL;
    plannerPort = openSerialPort(SERIAL_PORT_USART3,
        FUNCTION_PLANNER, 
        plannerDataRecieve, 
        &plannerRecieveBuffer,
        115200, 
        MODE_RXTX, 
        SERIAL_NOT_INVERTED);
    UNUSED(plannerPort);

    for (uint8_t i = 0; i < PLANNER_FRAME_SIZE; i++){
        plannerRecieveBuffer[i] = 0;
    }
}

/**
 * 以10hz的速度在调度器中更新，添加状态控制函数，防止意外状态导致飞机失控
 */
void plannerDataUnpack(timeUs_t currentTimeUs)
{
    union {
        uint8_t  recieveBuffer[PLANNER_FRAME_SIZE];
        uint16_t command[PLANNER_FRAME_SIZE / 2];
    } plannerInput;
    memcpy(plannerInput.recieveBuffer, plannerRecieveBuffer, PLANNER_FRAME_SIZE);

    timeMs_t currentTimeMs = currentTimeUs / 1000;
    bool updated = (currentTimeMs - latestRecieveTime < 1000) ? true : false;

    bool checkSumValid = false;
    uint16_t checkSum = 0;
    for (uint8_t i = 0; i < PLANNER_FRAME_SIZE/2 - 2; i++) {
        checkSum += plannerInput.command[i];
    }
    checkSum = ~checkSum;
    checkSumValid = (checkSum == plannerInput.command[4]);

    bool commandNotEmpty = (plannerInput.command[1] != 1500) || (plannerInput.command[2] != 1500) || (plannerInput.command[3] != 1500);

    bool avoidActive = IS_RC_MODE_ACTIVE(BOXUSER3) ? true : false;

     
    if (updated && checkSumValid && commandNotEmpty && avoidActive) {
        plannerCommandValidity = true;
        plannerCommand[0] = plannerInput.command[1];
        plannerCommand[1] = plannerInput.command[2];
        plannerCommand[2] = plannerInput.command[3];
    }
    else {
        plannerCommandValidity = false;
        plannerCommand[0] = 1500;
        plannerCommand[1] = 1500;
        plannerCommand[2] = 1500;
    }
}

bool isAvoidActive(void)
{
    return plannerCommandValidity;
}

int16_t plannerGetRcCommand(rc_alias_e axis)
{
    switch (axis) {
        case ROLL:
            return plannerCommand[0];
        case PITCH:
            return plannerCommand[1];
        case YAW:
            return plannerCommand[2];
        default:
            return 1500;
    }
}

// Receive ISR callback
void plannerDataRecieve(uint16_t byte, void *data)
{
    UNUSED(data);
    plannerRecieveBuffer[bufferIndex++] = byte;

    switch (state) {
        case WAIT_START:
            if (byte == FRAME_START_1) {
                state = START_STANDBY; 
                  
            }
            else{
                state = WAIT_START;
                bufferIndex = 0;
            }
            break;

        case START_STANDBY:
            if (byte == FRAME_START_2) {
                state = IN_FRAME;
            }
            else{
                state = WAIT_START;
                bufferIndex = 0;
            }
            break;

        case IN_FRAME:
            if (byte == FRAME_END_1) {
                state = END_STANDBY;
            }
            break;

        case END_STANDBY:
            if (byte == FRAME_END_2) {
                state = WAIT_START;
                bufferIndex = 0;
                latestRecieveTime = millis();
            } 
            else {
                state = IN_FRAME;
            }
            break;
        
        default:
            state = WAIT_START;
            bufferIndex = 0;
            break;
    }

    if (bufferIndex >= PLANNER_FRAME_SIZE) {
        state = WAIT_START;
        bufferIndex = 0;
    }
}