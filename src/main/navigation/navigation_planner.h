#pragma once
#include <stdbool.h>



void plannerInit(void);
void plannerDataRecieve(uint16_t c, void *data);
//void plannerDataRecieve(void);
void plannerDataUnpack(timeUs_t currentTimeUs);