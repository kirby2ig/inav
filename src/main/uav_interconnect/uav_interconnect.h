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

#include "platform.h"
#include "build/build_config.h"
#include "drivers/rangefinder.h"

#ifdef USE_UAV_INTERCONNECT

#define UIB_PACKET_SIZE 16

typedef enum {
    UIB_DATA_NONE           = 0,
    UIB_DATA_VALID          = (1 << 0),     // Data is valid
    UIB_DATA_NEW            = (1 << 1),     // Data has updated since last READ
} uibDataFlags_t;

/* Bus task */
bool uavInterconnectBusCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTime);
void uavInterconnectBusTask(timeUs_t currentTimeUs);
void uavInterconnectBusInit(void);
bool uavInterconnectBusIsInitialized(void);

/* Bus device API */
bool uibDeviceDetected(uint8_t devId);
bool uibGetDeviceParams(uint8_t devId, uint8_t * params);
timeUs_t uibGetPollRateUs(uint8_t devId);
uint32_t uibGetUnansweredRequests(uint8_t devId);
bool uibDataAvailable(uint8_t devId);
bool uibRead(uint8_t devId, uint8_t * buffer);

#define RANGEFINDER_UIB_TASK_PERIOD_MS  100
bool uibRangefinderDetect(rangefinderDev_t *dev);

#endif
