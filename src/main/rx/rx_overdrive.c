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

#ifdef USE_RX_OVERDRIVE

#include "build/build_config.h"

#include "common/utils.h"

#include "config/feature.h"

#include "rx_ros.h"
#include "rx/msp.h"

#include "fc/config.h"



uint16_t rxOverdriveRcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
uint8_t rxOverdriveNewPacketAvailable; // set true when a new packet is received

rxRuntimeConfig_t rxRuntimeConfigOverdriveMSP;
rxRuntimeConfig_t rxRuntimeConfigOverdriveROS;

uint16_t rxOverdriveReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
    uint16_t rM = rxRuntimeConfigOverdriveMSP.rcReadRawFn(rxRuntimeConfig, channel); // MSP
    uint16_t rR = rxRuntimeConfigOverdriveROS.rcReadRawFn(rxRuntimeConfig, channel); // ROS

    static uint32_t i=0;
    static uint16_t m[6] = {0};
    static uint16_t r[6] = {0};
    if(channel < 6) {
        m[channel] = rM;
        r[channel] = rR;
    }
    if(channel==55) {
        printf("RC#%i, %d %d %d %d %d %d | %d %d %d %d %d %d\n", i++
        , r[0], r[1], r[2], r[3], r[4], r[5]
        , m[0], m[1], m[2], m[3], m[4], m[5]
        );
    }

    return rR;
/*
    if (channel >= rxRuntimeConfig->channelCount) {
        return 0;
    }
    if (rxOverdriveNewPacketAvailable) {
//        protocolSetRcDataFromPayload(rxSpiRcData, rxSpiPayload);
        rxOverdriveNewPacketAvailable = false;
    }
    return rxOverdriveRcData[channel];*/
}


/*
 * Returns true if the RX has received new data.
 * Called from updateRx in rx.c, updateRx called from taskUpdateRxCheck.
 * If taskUpdateRxCheck returns true, then taskUpdateRxMain will shortly be called.
 */
static uint8_t rxOverdriveFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);
    volatile uint8_t rM = rxRuntimeConfigOverdriveMSP.rcFrameStatusFn(rxRuntimeConfig);
    uint8_t rR = rxRuntimeConfigOverdriveROS.rcFrameStatusFn(rxRuntimeConfig);
//    return RX_FRAME_COMPLETE;
    return rR;
/*
    if (true) {
        rxOverdriveNewPacketAvailable = true;
        return RX_FRAME_COMPLETE;
    }
    return RX_FRAME_PENDING;*/
}

/*
 * Set and initialize the RX protocol
 */
bool rxOverdriveInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    bool ret = true;

    rxOverdriveNewPacketAvailable = false;
    rxRuntimeConfig->rxRefreshRate = 20000;

    rxRuntimeConfig->rcReadRawFn = rxOverdriveReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = rxOverdriveFrameStatus;
    
    
    rxMspInit(rxConfig, &rxRuntimeConfigOverdriveMSP);
    rxRosInit(rxConfig, &rxRuntimeConfigOverdriveROS);

    return ret;
}
#endif
