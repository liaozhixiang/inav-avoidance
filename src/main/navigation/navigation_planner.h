#pragma once
#include <stdbool.h>



void plannerInit(void);
void plannerDataRecieve(uint16_t c, void *data);
void plannerDataUnpack(timeUs_t currentTimeUs);
int16_t plannerGetRcCommand(rc_alias_e axis);
bool isAvoidActive(void);