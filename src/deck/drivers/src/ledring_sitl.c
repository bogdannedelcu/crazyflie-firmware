/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ledring_sitl.c - SITL stub for the LED ring deck.
 * Exposes the same ring.* parameters as the real ledring12 driver
 * so that Python scripts (e.g. circling_square_demo_sitl.py) can
 * set LED colors without KeyError. Sends RGB color to the simulator
 * via CRTP so MuJoCo can render the LED colors on the drone model.
 */

#include <stdint.h>
#include <string.h>
#include "param.h"
#include "log.h"
#include "crtp.h"

/* LED color CRTP packet: SIM port, channel 1.
 * Byte 0: header (0x91)
 * Byte 1: solidRed
 * Byte 2: solidGreen
 * Byte 3: solidBlue
 */
#define CRTP_HDR_LED CRTP_HEADER(CRTP_PORT_SETPOINT_SIM, 1)

static uint8_t isInit = 1;

static uint8_t effect = 7;
static uint32_t neffect = 19;
static uint8_t solidRed = 0;
static uint8_t solidGreen = 0;
static uint8_t solidBlue = 0;
static uint8_t headlightEnable = 0;
static float emptyCharge = 3.1f;
static float fullCharge = 4.2f;
static uint32_t fadeColor = 0;
static float fadeTime = 0.0f;

static void sendLedColorToSim(void)
{
  CRTPPacket p;
  p.header = CRTP_HDR_LED;
  p.size = 3;
  p.data[0] = solidRed;
  p.data[1] = solidGreen;
  p.data[2] = solidBlue;
  crtpSendPacket(&p);
}

static void ledColorParamCallback(void)
{
  sendLedColorToSim();
}

/**
 * Parameters for the LED ring deck (SITL stub).
 */
PARAM_GROUP_START(ring)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_PERSISTENT, effect, &effect)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, solidRed, &solidRed, &ledColorParamCallback)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, solidGreen, &solidGreen, &ledColorParamCallback)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, solidBlue, &solidBlue, &ledColorParamCallback)
PARAM_ADD_CORE(PARAM_UINT8, headlightEnable, &headlightEnable)
PARAM_ADD_CORE(PARAM_FLOAT, emptyCharge, &emptyCharge)
PARAM_ADD_CORE(PARAM_FLOAT, fullCharge, &fullCharge)
PARAM_ADD_CORE(PARAM_UINT32, fadeColor, &fadeColor)
PARAM_ADD_CORE(PARAM_FLOAT, fadeTime, &fadeTime)

PARAM_GROUP_STOP(ring)

/**
 * LED ring LOG variables for SITL visualization.
 */
LOG_GROUP_START(ring)
LOG_ADD(LOG_UINT8, solidRed, &solidRed)
LOG_ADD(LOG_UINT8, solidGreen, &solidGreen)
LOG_ADD(LOG_UINT8, solidBlue, &solidBlue)
LOG_ADD(LOG_UINT8, effect, &effect)
LOG_GROUP_STOP(ring)
