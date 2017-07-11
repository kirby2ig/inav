/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include <string.h>

#include "platform.h"
#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#ifdef USE_UAV_INTERCONNECT
#include "drivers/rangefinder.h"
#include "uav_interconnect/uav_interconnect.h"

#define UIB_DEVICE_ADDRESS      0x12

#define RANGEFINDER_MAX_RANGE_CM                1000
#define RANGEFINDER_DETECTION_CONE_DECIDEGREES  450

typedef struct __attribute__((packed)) {
    uint8_t flags;
    uint16_t distanceCm;
} rangefinderData_t;

typedef union __attribute__((packed))  {
    uint8_t rawData[UIB_PACKET_SIZE];
    rangefinderData_t rangefinder;
} uibDataPacket_t;

static int32_t lastCalculatedDistance = RANGEFINDER_OUT_OF_RANGE;
static uibDataPacket_t uibData;

static void uibRangefinderInit(rangefinderDev_t *dev)
{
    UNUSED(dev);
}

static void uibRangefinderUpdate(rangefinderDev_t *dev)
{
    UNUSED(dev);

    if (!uavInterconnectBusIsInitialized()) {
        lastCalculatedDistance = RANGEFINDER_HARDWARE_FAILURE;
        return;
    }

    // If bus didn't detect the yet - report failure
    if (!uibDeviceDetected(UIB_DEVICE_ADDRESS)) {
        lastCalculatedDistance = RANGEFINDER_HARDWARE_FAILURE;
        return;
    }

    // If device is not responding - report failure
    if (uibGetUnansweredRequests(UIB_DEVICE_ADDRESS) > 0) {
        lastCalculatedDistance = RANGEFINDER_HARDWARE_FAILURE;
        return;
    }

    if (uibDataAvailable(UIB_DEVICE_ADDRESS)) {
        uibRead(UIB_DEVICE_ADDRESS, (uint8_t*)&uibData);

        if (!(uibData.rangefinder.flags & UIB_DATA_NEW))
            return;

        if (uibData.rangefinder.flags & UIB_DATA_VALID) {
            lastCalculatedDistance = uibData.rangefinder.distanceCm;
            dev->delayMs = uibGetPollRateUs(UIB_DEVICE_ADDRESS) / 1000;
        }
        else {
            lastCalculatedDistance = RANGEFINDER_OUT_OF_RANGE;
        }
    }
}

static int32_t uibRangefinderGetDistance(rangefinderDev_t *dev)
{
    UNUSED(dev);
    return lastCalculatedDistance;
}

bool uibRangefinderDetect(rangefinderDev_t *dev)
{
    // This always succeed
    dev->delayMs = RANGEFINDER_UIB_TASK_PERIOD_MS;
    dev->maxRangeCm = RANGEFINDER_MAX_RANGE_CM;
    dev->detectionConeDeciDegrees = RANGEFINDER_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = RANGEFINDER_DETECTION_CONE_DECIDEGREES;

    dev->init = &uibRangefinderInit;
    dev->update = &uibRangefinderUpdate;
    dev->read = &uibRangefinderGetDistance;

    return true;
}

#endif
