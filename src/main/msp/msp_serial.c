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

#include "build/debug.h"

#include "common/streambuf.h"
#include "common/utils.h"
#include "common/maths.h"

#include "drivers/serial.h"

#include "io/serial.h"

#include "msp/msp.h"
#include "msp/msp_serial.h"

static mspPort_t mspPorts[MAX_MSP_PORT_COUNT];


static void resetMspPort(mspPort_t *mspPortToReset, serialPort_t *serialPort)
{
    memset(mspPortToReset, 0, sizeof(mspPort_t));

    mspPortToReset->port = serialPort;
}

void mspSerialAllocatePorts(void)
{
    uint8_t portIndex = 0;
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MSP);
    while (portConfig && portIndex < MAX_MSP_PORT_COUNT) {
        mspPort_t *mspPort = &mspPorts[portIndex];
        if (mspPort->port) {
            portIndex++;
            continue;
        }

        serialPort_t *serialPort = openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL, baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
        if (serialPort) {
            resetMspPort(mspPort, serialPort);
            portIndex++;
        }

        portConfig = findNextSerialPortConfig(FUNCTION_MSP);
    }
}

void mspSerialReleasePortIfAllocated(serialPort_t *serialPort)
{
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t *candidateMspPort = &mspPorts[portIndex];
        if (candidateMspPort->port == serialPort) {
            closeSerialPort(serialPort);
            memset(candidateMspPort, 0, sizeof(mspPort_t));
        }
    }
}

static bool mspSerialProcessReceivedData(mspPort_t *mspPort, uint8_t c)
{
    switch (mspPort->c_state) {
        default:
        case MSP_IDLE:      // Waiting for '$' character
            if (c == '$') {
                mspPort->isMSPv2 = false;
                mspPort->c_state = MSP_HEADER_START;
            }
            else {
                return false;
            }
            break;

        case MSP_HEADER_START:  // Waiting for 'M'
            mspPort->c_state = (c == 'M') ? MSP_HEADER_M : MSP_IDLE;
            break;

        case MSP_HEADER_M:      // Waiting for '<'
            if (c == '<') {
                mspPort->offset = 0;
                mspPort->checksum1 = 0;
                mspPort->checksum2 = 0;
                mspPort->c_state = MSP_HEADER_V1;
            }
            else {
                mspPort->c_state = MSP_IDLE;
            }
            break;

        case MSP_HEADER_V1:     // Now receive v1 header (size/cmd), this is already checksummable
            mspPort->inBuf[mspPort->offset++] = c;
            mspPort->checksum1 ^= c;
            if (mspPort->offset == sizeof(mspHeaderV1_t)) {
                mspHeaderV1_t * hdr = (mspHeaderV1_t *)&mspPort->inBuf[0];
                // Check incoming buffer size limit
                if (hdr->size > MSP_PORT_INBUF_SIZE) {
                    mspPort->c_state = MSP_IDLE;
                }
                else if (hdr->cmd == MSP_V2_FRAME_ID) {
                    // MSPv1 payload must be big enough to hold V2 header + extra checksum
                    if (hdr->size >= sizeof(mspHeaderV2_t) + 1) {
                        mspPort->isMSPv2 = true;
                        mspPort->c_state = MSP_HEADER_V2;
                    }
                    else {
                        mspPort->c_state = MSP_IDLE;
                    }
                }
                else {
                    mspPort->dataSize = hdr->size;
                    mspPort->cmdMSP = hdr->cmd;
                    mspPort->offset = 0;                // re-use buffer
                    mspPort->c_state = mspPort->dataSize > 0 ? MSP_PAYLOAD_V1 : MSP_CHECKSUM_V1;    // If no payload - jump to checksum byte
                }
            }
            break;

        case MSP_PAYLOAD_V1:
            mspPort->inBuf[mspPort->offset++] = c;
            mspPort->checksum1 ^= c;
            if (mspPort->offset == mspPort->dataSize) {
                mspPort->c_state = MSP_CHECKSUM_V1;
            }
            break;

        case MSP_CHECKSUM_V1:
            if (mspPort->checksum1 == c) {
                mspPort->c_state = MSP_COMMAND_RECEIVED;
            } else {
                mspPort->c_state = MSP_IDLE;
            }
            break;

        case MSP_HEADER_V2:     // V2 header is part of V1 payload - we need to calculate both checksums now
            mspPort->inBuf[mspPort->offset++] = c;
            mspPort->checksum1 ^= c;
            mspPort->checksum2 = crc8_dvb_s2(mspPort->checksum2, c);
            if (mspPort->offset == (sizeof(mspHeaderV2_t) + sizeof(mspHeaderV1_t))) {
                mspHeaderV1_t * hdrv1 = (mspHeaderV1_t *)&mspPort->inBuf[0];
                mspHeaderV2_t * hdrv2 = (mspHeaderV2_t *)&mspPort->inBuf[sizeof(mspHeaderV1_t)];

                mspPort->dataSize = hdrv1->size - sizeof(mspHeaderV2_t) - 1;    // V1 size - V1 header - extra checksum byte
                mspPort->cmdMSP = hdrv2->cmd;
                mspPort->offset = 0;                // re-use buffer
                mspPort->c_state = mspPort->dataSize > 0 ? MSP_PAYLOAD_V2 : MSP_CHECKSUM_V2;
            }
            break;

        case MSP_PAYLOAD_V2:
            mspPort->checksum2 = crc8_dvb_s2(mspPort->checksum2, c);
            mspPort->checksum1 ^= c;
            mspPort->inBuf[mspPort->offset++] = c;

            if (mspPort->offset == mspPort->dataSize) {
                mspPort->c_state = MSP_CHECKSUM_V2;
            }
            break;

        case MSP_CHECKSUM_V2:
            mspPort->checksum1 ^= c;
            if (mspPort->checksum2 == c) {
                mspPort->c_state = MSP_CHECKSUM_V1; // Checksum 2 correct - verify v1 checksum
            } else {
                mspPort->c_state = MSP_IDLE;
            }
            break;
    }

    return true;
}

static uint8_t mspSerialChecksumBuf(uint8_t checksum, const uint8_t *data, int len)
{
    while (len-- > 0) {
        checksum ^= *data++;
    }
    return checksum;
}

static uint8_t mspSerialChecksumBufV2(uint8_t checksum, const uint8_t *data, int len)
{
    while (len-- > 0) {
        checksum = crc8_dvb_s2(checksum, *data++);
    }
    return checksum;
}

#define JUMBO_FRAME_SIZE_LIMIT 255

static int mspSerialEncode(mspPort_t *msp, mspPacket_t *packet, bool isMSPv2)
{
    const int dataLen = sbufBytesRemaining(&packet->buf);
    const int v1PayloadSize = isMSPv2 ? dataLen + (int)sizeof(mspHeaderV2_t) + 1 : dataLen;
    uint8_t hdrBuf[10] = {'$', 'M', packet->result == MSP_RESULT_ERROR ? '!' : '>'};
    int hdrLen = 3;

    // Reserve space for V1 header
    mspHeaderV1_t *     hdrV1 = (mspHeaderV1_t *)&hdrBuf[hdrLen];
    hdrLen += sizeof(mspHeaderV1_t);

    // Add JUMBO-frame header if necessary
    if (v1PayloadSize >= JUMBO_FRAME_SIZE_LIMIT) {
        mspHeaderJUMBO_t * hdrJUMBO = (mspHeaderJUMBO_t *)&hdrBuf[hdrLen];
        hdrLen += sizeof(mspHeaderJUMBO_t);

        hdrV1->size = JUMBO_FRAME_SIZE_LIMIT;
        hdrJUMBO->size = v1PayloadSize;
    }
    else {
        hdrV1->size = v1PayloadSize;
    }

    // Add V2 header if this should be a V2 packet and send packet-
    mspHeaderV2_t * hdrV2 = NULL;
    if (isMSPv2) {
        hdrV2 = (mspHeaderV2_t *)&hdrBuf[hdrLen];
        hdrLen += sizeof(mspHeaderV2_t);

        hdrV1->cmd = MSP_V2_FRAME_ID;
        hdrV2->cmd = packet->cmd;
    }
    else {
        hdrV1->cmd = packet->cmd;
    }

    // We are allowed to send out the response if
    //  a) TX buffer is completely empty (we are talking to well-behaving party that follows request-response scheduling;
    //     this allows us to transmit jumbo frames bigger than TX buffer (serialWriteBuf will block, but for jumbo frames we don't care)
    //  b) Response fits into TX buffer
    const int totalFrameLength = isMSPv2 ? hdrLen + dataLen + 2 : hdrLen + dataLen + 1;
    if (!isSerialTransmitBufferEmpty(msp->port) && ((int)serialTxBytesFree(msp->port) < totalFrameLength))
        return 0;

    #define V1_CHECKSUM_STARTPOS 3

    // Transmit headers
    serialBeginWrite(msp->port);
    serialWriteBuf(msp->port, hdrBuf, hdrLen);

    // Now calculate V1 checksum and send data payload
    uint8_t checksum1 = mspSerialChecksumBuf(0, hdrBuf + V1_CHECKSUM_STARTPOS, hdrLen - V1_CHECKSUM_STARTPOS);
    if (dataLen > 0) {
        serialWriteBuf(msp->port, sbufPtr(&packet->buf), dataLen);
        checksum1 = mspSerialChecksumBuf(checksum1, sbufPtr(&packet->buf), dataLen);
    }

    // For MSPv2 we need to send additional checksum - V2 header + data payload. Note that V2 checksum is part of V1 payload
    if (isMSPv2) {
        uint8_t checksum2 = mspSerialChecksumBufV2(0, (uint8_t *)hdrV2, sizeof(mspHeaderV2_t));
        checksum2 = mspSerialChecksumBufV2(checksum2, sbufPtr(&packet->buf), dataLen);
        checksum1 ^= checksum2;
        serialWriteBuf(msp->port, &checksum2, 1);
    }

    serialWriteBuf(msp->port, &checksum1, 1);
    serialEndWrite(msp->port);
    return totalFrameLength; // header, data, and checksum
}

static mspPostProcessFnPtr mspSerialProcessReceivedCommand(mspPort_t *msp, mspProcessCommandFnPtr mspProcessCommandFn)
{
    static uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];

    mspPacket_t reply = {
        .buf = { .ptr = outBuf, .end = ARRAYEND(outBuf), },
        .cmd = -1,
        .result = 0,
    };
    uint8_t *outBufHead = reply.buf.ptr;

    mspPacket_t command = {
        .buf = { .ptr = msp->inBuf, .end = msp->inBuf + msp->dataSize, },
        .cmd = msp->cmdMSP,
        .result = 0,
    };

    mspPostProcessFnPtr mspPostProcessFn = NULL;
    const mspResult_e status = mspProcessCommandFn(&command, &reply, &mspPostProcessFn);

    if (status != MSP_RESULT_NO_REPLY) {
        sbufSwitchToReader(&reply.buf, outBufHead); // change streambuf direction
        mspSerialEncode(msp, &reply, msp->isMSPv2);
    }

    msp->c_state = MSP_IDLE;
    return mspPostProcessFn;
}

/*
 * Process MSP commands from serial ports configured as MSP ports.
 *
 * Called periodically by the scheduler.
 */
void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData, mspProcessCommandFnPtr mspProcessCommandFn)
{
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }
        mspPostProcessFnPtr mspPostProcessFn = NULL;
        while (serialRxBytesWaiting(mspPort->port)) {

            const uint8_t c = serialRead(mspPort->port);
            const bool consumed = mspSerialProcessReceivedData(mspPort, c);

            if (!consumed && evaluateNonMspData == MSP_EVALUATE_NON_MSP_DATA) {
                serialEvaluateNonMspData(mspPort->port, c);
            }

            if (mspPort->c_state == MSP_COMMAND_RECEIVED) {
                mspPostProcessFn = mspSerialProcessReceivedCommand(mspPort, mspProcessCommandFn);
                break; // process one command at a time so as not to block.
            }
        }
        if (mspPostProcessFn) {
            waitForSerialPortToFinishTransmitting(mspPort->port);
            mspPostProcessFn(mspPort->port);
        }
    }
}

void mspSerialInit(void)
{
    memset(mspPorts, 0, sizeof(mspPorts));
    mspSerialAllocatePorts();
}

int mspSerialPush(uint8_t cmd, const uint8_t *data, int datalen)
{
    static uint8_t pushBuf[30];
    int ret = 0;

    mspPacket_t push = {
        .buf = { .ptr = pushBuf, .end = ARRAYEND(pushBuf), },
        .cmd = cmd,
        .result = 0,
    };

    for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }

        // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
        if (mspPort->port->identifier == SERIAL_PORT_USB_VCP) {
            continue;
        }

        sbufWriteData(&push.buf, data, datalen);

        sbufSwitchToReader(&push.buf, pushBuf);

        ret = mspSerialEncode(mspPort, &push, false);
    }
    return ret; // return the number of bytes written
}

uint32_t mspSerialTxBytesFree()
{
    uint32_t ret = UINT32_MAX;

    for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }

        // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
        if (mspPort->port->identifier == SERIAL_PORT_USB_VCP) {
            continue;
        }

        const uint32_t bytesFree = serialTxBytesFree(mspPort->port);
        if (bytesFree < ret) {
            ret = bytesFree;
        }
    }

    return ret;
}
