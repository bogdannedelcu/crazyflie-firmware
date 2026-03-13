/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * led_sitl.c - SITL stub for LED handling functions.
 * Stores LED state in memory and exposes it via LOG variables
 * so simulators can read and visualize the LED states.
 */
#include <stdbool.h>

#include "led.h"
#include "param.h"
#include "log.h"

#define LED_ENABLE_BITMASK_BIT    7

static bool isInit = false;
static uint8_t ledControlBitmask;
static uint8_t ledState[LED_NUM];
ledSwitch_t ledSwitchState;

static void ledSetSwitch(ledSwitch_t ledSwitch)
{
  if (ledSwitchState != ledSwitch)
  {
    ledSwitchState = ledSwitch;
  }
}

static void ledBitmaskParamCallback(void)
{
  if (ledControlBitmask & (1 << LED_ENABLE_BITMASK_BIT))
  {
    ledSetSwitch(LED_PARAM_BITMASK);
    for (int i = 0; i < LED_NUM; i++)
    {
      ledState[i] = (ledControlBitmask & (1 << i)) ? 1 : 0;
    }
  }
  else
  {
    ledSetSwitch(LED_LEDSEQ);
  }
}

void ledInit()
{
  if (isInit)
    return;

  for (int i = 0; i < LED_NUM; i++)
  {
    ledState[i] = 0;
  }

  ledSwitchState = LED_LEDSEQ;
  isInit = true;
}

bool ledTest(void)
{
  return isInit;
}

void ledClearAll(void)
{
  for (int i = 0; i < LED_NUM; i++)
  {
    ledState[i] = 0;
  }
}

void ledSetAll(void)
{
  for (int i = 0; i < LED_NUM; i++)
  {
    ledState[i] = 1;
  }
}

void ledSet(led_t led, bool value)
{
  if (led >= LED_NUM)
  {
    return;
  }

  ledState[led] = value ? 1 : 0;
}

void ledShowFaultPattern(void)
{
  ledSet(LED_GREEN_L, 0);
  ledSet(LED_GREEN_R, 0);
  ledSet(LED_RED_L, 1);
  ledSet(LED_RED_R, 1);
  ledSet(LED_BLUE_L, 0);
}

/**
 * Parameters governing the onboard LEDs
 * */
PARAM_GROUP_START(led)
/**
 * @brief Control onboard LEDs using a bitmask. Enabling it will override the led sequencer.
 *
 * ```
 * | 7:ENABLE | 6:N/A | 5:BLUE_R | 4:RED_R | 3:GREEN_R | 2:RED_L | 1:GREEN_L | 0:BLUE_L |
 * ```
 */
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, bitmask, &ledControlBitmask, &ledBitmaskParamCallback)

PARAM_GROUP_STOP(led)

/**
 * LED state log variables for SITL visualization.
 * Each variable is 1 (on) or 0 (off).
 */
LOG_GROUP_START(led)
LOG_ADD(LOG_UINT8, blueL, &ledState[LED_BLUE_L])
LOG_ADD(LOG_UINT8, greenL, &ledState[LED_GREEN_L])
LOG_ADD(LOG_UINT8, redL, &ledState[LED_RED_L])
LOG_ADD(LOG_UINT8, greenR, &ledState[LED_GREEN_R])
LOG_ADD(LOG_UINT8, redR, &ledState[LED_RED_R])
LOG_GROUP_STOP(led)
