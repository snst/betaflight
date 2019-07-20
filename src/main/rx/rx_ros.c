/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RX_ROS

#include "build/build_config.h"

#include "common/utils.h"

#include "config/feature.h"

#include "rx_ros.h"

#include "fc/config.h"
#include "fcl_types.h"

uint16_t rxRosRcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
uint8_t rxRosNewPacketAvailable; // set true when a new packet is received
static fcl_joystick_t js;

uint16_t calcAxisVal(float val) {
  return 1000 + (500.0f * (val + 1.0)); //
}

uint16_t calcButtonVal(uint16_t val, uint16_t btn) {
  return ((val & btn) > 0) ? 1700 : 1300; //
}

uint16_t rxRosReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig,
                        uint8_t channel) {
  uint16_t ret = 0;
  switch (channel) {
  case 0:
    ret = calcAxisVal(js.val[0]);
    break;
  case 1:
    ret = calcAxisVal(-js.val[1]);
    break;
  case 2:
    ret = calcAxisVal(-js.val[2]); // throttle
    break;
  case 3:
    ret = calcAxisVal(js.val[4]);
    break;
  case 4:
    ret = calcAxisVal(js.val[3]);
    break;
  case 5:
    ret = calcButtonVal(js.button, 1);
    break;
  case 6:
    ret = calcButtonVal(js.button, 2); 
    break;
  case 7:
    ret = calcButtonVal(js.button, 4);
    break;
  case 8:
    ret = calcButtonVal(js.button, 8);
    break;
  case 9:
    ret = calcButtonVal(js.button, 16);
    break;
  }

  //  printf("js %u: %f %u\n", channel, val, ret);

  return ret;
  /*
  if (channel >= rxRuntimeConfig->channelCount) {
      return 0;
  }
  if (rxRosNewPacketAvailable) {
//        protocolSetRcDataFromPayload(rxSpiRcData, rxSpiPayload);
      rxRosNewPacketAvailable = false;
  }
  return rxRosRcData[channel];*/
}

/*
 * Returns true if the RX has received new data.
 * Called from updateRx in rx.c, updateRx called from taskUpdateRxCheck.
 * If taskUpdateRxCheck returns true, then taskUpdateRxMain will shortly be
 * called.
 */
static uint8_t rxRosFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig) {
  UNUSED(rxRuntimeConfig);
  static uint32_t l = 0;
  fcl_get_from_sim(eJoystick, &js);

    if ((js.button & 4) > 0) {
        printf("reset\n");
        fcl_resetworld_t r;
        fcl_send_to_sim(eResetWorld, &r);
    }

 // printf("%u: %f %f %f %f %f %u\n", l++, js.val[0], js.val[1], js.val[2], js.val[3],
 //        js.val[4], js.button);

  static uint32_t last_ms = 0;

  uint32_t now = millis();
  if (now - last_ms > 100) {
    last_ms = now;
    return RX_FRAME_COMPLETE;
  }

  return RX_FRAME_PENDING;
}

/*
 * Set and initialize the RX protocol
 */
bool rxRosInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig) {
  bool ret = true;

  rxRosNewPacketAvailable = false;
  rxRuntimeConfig->rxRefreshRate = 20000;

  rxRuntimeConfig->rcReadRawFn = rxRosReadRawRC;
  rxRuntimeConfig->rcFrameStatusFn = rxRosFrameStatus;

  return ret;
}
#endif
