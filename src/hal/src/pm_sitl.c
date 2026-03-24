#include <stdint.h>
#include "log.h"

// Non-static so sensors_sitl.c can update these when battery simulation is active
float simBatteryVoltage = 4.2f;
uint16_t simBatteryVoltageMV = 4200;
int8_t simPmState = 0;

LOG_GROUP_START(pm)
LOG_ADD_CORE(LOG_FLOAT, vbat, &simBatteryVoltage)
LOG_ADD(LOG_UINT16, vbatMV, &simBatteryVoltageMV)
LOG_ADD_CORE(LOG_INT8, state, &simPmState)
LOG_GROUP_STOP(pm)
