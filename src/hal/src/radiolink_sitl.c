/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie simulation firmware
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
 * radiolink_sitl.c - Radio link layer SITL stub
 */

#include <stdint.h>
#include "log.h"

static uint8_t rssi = 100;
static uint8_t isConnected = 1;
static uint16_t count_rx_broadcast = 0;
static uint16_t count_rx_unicast = 0;

LOG_GROUP_START(radio)
/**
 * @brief Radio Signal Strength Indicator [dBm]
 */
LOG_ADD_CORE(LOG_UINT8, rssi, &rssi)
/**
 * @brief Indicator if a packet was received from the radio within the last RADIO_ACTIVITY_TIMEOUT_MS.
 */
LOG_ADD_CORE(LOG_UINT8, isConnected, &isConnected)
/**
 * @brief Number of broadcast packets received.
 * 
 * Note that this is only 16 bits and overflows. Use overflow correction on the client side.
 */
LOG_ADD_CORE(LOG_UINT16, numRxBc, &count_rx_broadcast)
/**
 * @brief Number of unicast packets received.
 * 
 * Note that this is only 16 bits and overflows. Use overflow correction on the client side.
 */
LOG_ADD_CORE(LOG_UINT16, numRxUc, &count_rx_unicast)
LOG_GROUP_STOP(radio)
