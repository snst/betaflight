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

#if defined(USE_FAKE_RANGEFINDER)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/time.h"

#include "drivers/time.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_SITL.h"


#define SITL_RANGEFINDER_MAX_RANGE_CM 400 // 4m, from HC-SR04 spec sheet
#define SITL_RANGEFINDER_DETECTION_CONE_DECIDEGREES 300 // recommended cone angle30 degrees, from HC-SR04 spec sheet
#define SITL_RANGEFINDER_DETECTION_CONE_EXTENDED_DECIDEGREES 450 // in practice 45 degrees seems to work well


int32_t SITL_lastCalculatedDistance = RANGEFINDER_OUT_OF_RANGE;
static timeMs_t lastMeasurementStartedAt = 0;

int get_sonar_range();

void sitl_rangefinder_init(rangefinderDev_t *dev)
{
    UNUSED(dev);
}

#define SITL_RANGEFINDER_MinimumFiringIntervalMs 60


void sitl_rangefinder_update(rangefinderDev_t *dev)
{
    UNUSED(dev);
    const timeMs_t timeNowMs = millis();

    // the firing interval of the trigger signal should be greater than 60ms
    // to avoid interference between consecutive measurements
    if (timeNowMs > lastMeasurementStartedAt + SITL_RANGEFINDER_MinimumFiringIntervalMs) {

        SITL_lastCalculatedDistance = get_sonar_range();
;

        // Trigger a new measurement
        lastMeasurementStartedAt = timeNowMs;
    }
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t sitl_rangefinder_get_distance(rangefinderDev_t *dev)
{
    UNUSED(dev);
    return SITL_lastCalculatedDistance;
}

bool rangefinderSitlDetect(rangefinderDev_t *dev)
{
    bool detected = true;

    if (detected) {
        dev->delayMs = 100;
        dev->maxRangeCm = SITL_RANGEFINDER_MAX_RANGE_CM;
        dev->detectionConeDeciDegrees = SITL_RANGEFINDER_DETECTION_CONE_DECIDEGREES;
        dev->detectionConeExtendedDeciDegrees = SITL_RANGEFINDER_DETECTION_CONE_EXTENDED_DECIDEGREES;

        dev->init = &sitl_rangefinder_init;
        dev->update = &sitl_rangefinder_update;
        dev->read = &sitl_rangefinder_get_distance;

        return true;
    }
    else {
        // Not detected - free resources
        return false;
    }
}

#endif
