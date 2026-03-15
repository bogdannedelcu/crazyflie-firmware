/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
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
 * colorled_sitl.c - SITL stub for the Color LED deck and LED ring params.
 * Exposes colorLedBot/Top parameters so cfclient's Color LED tab works,
 * and ring.* parameters for script compatibility.
 * Sends RGB color to the simulator via CRTP for MuJoCo visualization.
 */

#include <stdint.h>
#include "param.h"
#include "log.h"
#include "crtp.h"

/* LED color CRTP packet: SIM port, channel 1.
 * Byte 0: header (0x91)
 * Byte 1: position (0=bot, 1=top)
 * Byte 2: R
 * Byte 3: G
 * Byte 4: B
 */
#define CRTP_HDR_LED CRTP_HEADER(CRTP_PORT_SETPOINT_SIM, 1)

/* Headlight CRTP packet: SIM port, channel 2. */
#define CRTP_HDR_HEADLIGHT CRTP_HEADER(CRTP_PORT_SETPOINT_SIM, 2)

#define LED_POS_BOT 0
#define LED_POS_TOP 1

static void sendColorToSim(uint8_t position, uint32_t wrgb)
{
  uint8_t w = (wrgb >> 24) & 0xFF;
  uint8_t r = (wrgb >> 16) & 0xFF;
  uint8_t g = (wrgb >> 8)  & 0xFF;
  uint8_t b =  wrgb        & 0xFF;

  /* Add white channel back to get full RGB */
  uint16_t rFull = r + w;
  uint16_t gFull = g + w;
  uint16_t bFull = b + w;
  if (rFull > 255) rFull = 255;
  if (gFull > 255) gFull = 255;
  if (bFull > 255) bFull = 255;

  CRTPPacket p;
  p.header = CRTP_HDR_LED;
  p.size = 4;
  p.data[0] = position;
  p.data[1] = (uint8_t)rFull;
  p.data[2] = (uint8_t)gFull;
  p.data[3] = (uint8_t)bFull;
  crtpSendPacket(&p);
}

/* ---- Color LED deck (bottom) ---- */

static uint32_t wrgb8888Bot = 0;
static uint8_t brightCorrBot = 100;
static uint8_t throttlePctBot = 0;

static void colorBotCallback(void)
{
  sendColorToSim(LED_POS_BOT, wrgb8888Bot);
}

PARAM_GROUP_START(colorLedBot)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT32, wrgb8888, &wrgb8888Bot, &colorBotCallback)
PARAM_ADD_CORE(PARAM_UINT8, brightCorr, &brightCorrBot)
PARAM_GROUP_STOP(colorLedBot)

LOG_GROUP_START(colorLedBot)
LOG_ADD(LOG_UINT8, throttlePct, &throttlePctBot)
LOG_GROUP_STOP(colorLedBot)

/* ---- Color LED deck (top) ---- */

static uint32_t wrgb8888Top = 0;
static uint8_t brightCorrTop = 100;
static uint8_t throttlePctTop = 0;

static void colorTopCallback(void)
{
  sendColorToSim(LED_POS_TOP, wrgb8888Top);
}

PARAM_GROUP_START(colorLedTop)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT32, wrgb8888, &wrgb8888Top, &colorTopCallback)
PARAM_ADD_CORE(PARAM_UINT8, brightCorr, &brightCorrTop)
PARAM_GROUP_STOP(colorLedTop)

LOG_GROUP_START(colorLedTop)
LOG_ADD(LOG_UINT8, throttlePct, &throttlePctTop)
LOG_GROUP_STOP(colorLedTop)

/* ---- LED ring params (for script compatibility) ---- */

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

static void ledColorParamCallback(void)
{
  /* Ring params set both top and bottom */
  uint32_t wrgb = ((uint32_t)solidRed << 16) |
                  ((uint32_t)solidGreen << 8) |
                  (uint32_t)solidBlue;
  sendColorToSim(LED_POS_BOT, wrgb);
  sendColorToSim(LED_POS_TOP, wrgb);
}

static void headlightParamCallback(void)
{
  CRTPPacket p;
  p.header = CRTP_HDR_HEADLIGHT;
  p.size = 1;
  p.data[0] = headlightEnable;
  crtpSendPacket(&p);
}

PARAM_GROUP_START(ring)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_PERSISTENT, effect, &effect)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, solidRed, &solidRed, &ledColorParamCallback)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, solidGreen, &solidGreen, &ledColorParamCallback)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, solidBlue, &solidBlue, &ledColorParamCallback)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, headlightEnable, &headlightEnable, &headlightParamCallback)
PARAM_ADD_CORE(PARAM_FLOAT, emptyCharge, &emptyCharge)
PARAM_ADD_CORE(PARAM_FLOAT, fullCharge, &fullCharge)
PARAM_ADD_CORE(PARAM_UINT32, fadeColor, &fadeColor)
PARAM_ADD_CORE(PARAM_FLOAT, fadeTime, &fadeTime)
PARAM_GROUP_STOP(ring)

LOG_GROUP_START(ring)
LOG_ADD(LOG_UINT8, solidRed, &solidRed)
LOG_ADD(LOG_UINT8, solidGreen, &solidGreen)
LOG_ADD(LOG_UINT8, solidBlue, &solidBlue)
LOG_ADD(LOG_UINT8, effect, &effect)
LOG_GROUP_STOP(ring)
