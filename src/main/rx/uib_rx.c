/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_UAV_INTERCONNECT) && defined(USE_RX_UIB)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/utils.h"
#include "common/maths.h"

#include "rx/rx.h"
#include "rx/uib_rx.h"

#include "uav_interconnect/uav_interconnect.h"

#define UIB_DEVICE_ADDRESS      0x80

typedef struct __attribute__((packed)) {
    uint8_t  flags;         // UIB_DATA_VALID (0x01) - link ok, UIB_DATA_NEW (0x02) - new data
    uint8_t  rssi;
    uint16_t sticks[4];     // Values in range [1000;2000], center = 1500
    uint8_t  aux[6];        // Values in range [0;255], center = 127
} rcReceiverData_t;

typedef union __attribute__((packed)) {
    uint8_t rawData[UIB_PACKET_SIZE];
    rcReceiverData_t rc;
} uibDataPacket_t;

static uibDataPacket_t uibData;

#define UIB_RX_MAX_CHANNEL_COUNT    11

static uint16_t rxUIBReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfigPtr, uint8_t chan)
{
    UNUSED(rxRuntimeConfigPtr);

    if (chan < 4) {
        return constrain(uibData.rc.sticks[chan], 1000, 2000);
    }
    else if (chan < 10) {
        return scaleRange(uibData.rc.sticks[chan - 4], 0, 255, 1000, 2000);
    }
    else {
        return scaleRange(uibData.rc.rssi, 0, 255, 1000, 2000);
    }
}

static uint8_t rxUIBFrameStatus(void)
{
    if (!uavInterconnectBusIsInitialized()) {
        return RX_FRAME_FAILSAFE;
    }

    // If bus didn't detect the yet - report failure
    if (!uibDeviceDetected(UIB_DEVICE_ADDRESS)) {
        return RX_FRAME_FAILSAFE;
    }

    if (uibGetUnansweredRequests(UIB_DEVICE_ADDRESS) > 10) {         // Tolerate 200ms loss (10 packet loss)
        return RX_FRAME_FAILSAFE;
    }

    if (uibDataAvailable(UIB_DEVICE_ADDRESS)) {
        uibDataPacket_t uibDataTmp;
        uibRead(UIB_DEVICE_ADDRESS, (uint8_t*)&uibDataTmp);

        if (!(uibDataTmp.rc.flags & UIB_DATA_VALID))
            return RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;

        if (!(uibDataTmp.rc.flags & UIB_DATA_NEW))
            return RX_FRAME_PENDING;

        memcpy(&uibData, &uibDataTmp, sizeof(uibDataPacket_t));
        return RX_FRAME_COMPLETE;
    }
    else {
        return RX_FRAME_PENDING;
    }
}

void rxUIBInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);

    rxRuntimeConfig->channelCount = UIB_RX_MAX_CHANNEL_COUNT;
    rxRuntimeConfig->rxRefreshRate = 20000;
    rxRuntimeConfig->rcReadRawFn = rxUIBReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = rxUIBFrameStatus;
}
#endif
