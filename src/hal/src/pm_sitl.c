#include <stdint.h>
#include "log.h"

static float batteryVoltage = 4.2f;
static uint16_t batteryVoltageMV = 4200;
static int8_t pmState = 0;

LOG_GROUP_START(pm)
LOG_ADD_CORE(LOG_FLOAT, vbat, &batteryVoltage)
LOG_ADD(LOG_UINT16, vbatMV, &batteryVoltageMV)
LOG_ADD_CORE(LOG_INT8, state, &pmState)
LOG_GROUP_STOP(pm)
