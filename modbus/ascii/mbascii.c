/*
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbascii.c,v 1.17 2010/06/06 13:47:07 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"
#include "mbascii.h"
#include "mbframe.h"

#include "mbcrc.h"
#include "mbport.h"

#if MB_ASCII_ENABLED > 0

/* ----------------------- Defines ------------------------------------------*/
#define MB_ASCII_DEFAULT_CR     '\r'    /*!< Default CR character for Modbus ASCII. */
#define MB_ASCII_DEFAULT_LF     '\n'    /*!< Default LF character for Modbus ASCII. */
#define MB_SER_PDU_SIZE_MIN     3       /*!< Minimum size of a Modbus ASCII frame. */
#define MB_SER_PDU_SIZE_MAX     255     /*!< Maximum size of a Modbus ASCII frame. */
#define MB_SER_PDU_SIZE_LRC     1       /*!< Size of LRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_WAIT_EOF           /*!< Wait for End of Frame. */
} eMBRcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_START,             /*!< Starting transmission (':' sent). */
    STATE_TX_DATA,              /*!< Sending of data (Address, Data, LRC). */
    STATE_TX_END,               /*!< End of transmission. */
    STATE_TX_NOTIFY             /*!< Notify sender that the frame has been sent. */
} eMBSndState;

typedef enum
{
    BYTE_HIGH_NIBBLE,           /*!< Character for high nibble of byte. */
    BYTE_LOW_NIBBLE             /*!< Character for low nibble of byte. */
} eMBBytePos;

/* ----------------------- Static functions ---------------------------------*/
static uint8_t      prvucMBCHAR2BIN(uint8_t ucCharacter);
static uint8_t      prvucMBBIN2CHAR(uint8_t ucByte);
static uint8_t      prvucMBLRC(uint8_t *pucFrame, uint16_t usLen);

/* ----------------------- Static variables ---------------------------------*/
static volatile eMBSndState eSndState;
static volatile eMBRcvState eRcvState;

/* We reuse the Modbus RTU buffer because only one buffer is needed and the
 * RTU buffer is bigger. */
extern volatile uint8_t     ucRTUBuf[];
#define ucASCIIBuf          ucRTUBuf

static volatile uint16_t    usRcvBufferPos;
static volatile eMBBytePos  eBytePos;

static volatile uint8_t    *pucSndBufferCur;
static volatile uint16_t    usSndBufferCount;

static volatile uint8_t     ucLRC;
static volatile uint8_t     ucMBLFCharacter;

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBASCIIInit(uint8_t ucSlaveAddress, uint8_t ucPort, uint32_t ulBaudRate, eMBParity eParity)
{
    eMBErrorCode    eStatus = MB_ENOERR;

    (void)ucSlaveAddress;

    ENTER_CRITICAL_SECTION();
    ucMBLFCharacter = MB_ASCII_DEFAULT_LF;

    if (xMBPortSerialInit(ucPort, ulBaudRate, 7, eParity) != TRUE) {
        eStatus = MB_EPORTERR;
    } else if (xMBPortTimersInit(MB_ASCII_TIMEOUT_SEC * 20000UL) != TRUE) {
        eStatus = MB_EPORTERR;
    }

    EXIT_CRITICAL_SECTION();

    return eStatus;
}

void
eMBASCIIStart(void)
{
    ENTER_CRITICAL_SECTION();
    vMBPortSerialEnable(TRUE, FALSE);
    eRcvState = STATE_RX_IDLE;
    EXIT_CRITICAL_SECTION();

    /* No special startup required for ASCII. */
    (void)xMBPortEventPost(EV_READY);
}

void
eMBASCIIStop(void)
{
    ENTER_CRITICAL_SECTION();
    vMBPortSerialEnable(FALSE, FALSE);
    vMBPortTimersDisable();
    EXIT_CRITICAL_SECTION();
}

eMBErrorCode
eMBASCIIReceive(uint8_t *pucRcvAddress, uint8_t **pucFrame, uint16_t *pusLength)
{
    eMBErrorCode    eStatus = MB_ENOERR;

    assert(usRcvBufferPos <= MB_SER_PDU_SIZE_MAX);
    ENTER_CRITICAL_SECTION();

    /* Length and CRC check */
    if ((usRcvBufferPos >= MB_SER_PDU_SIZE_MIN) &&
        (prvucMBLRC((uint8_t *) ucASCIIBuf, usRcvBufferPos) == 0)) {
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = ucASCIIBuf[MB_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = (uint16_t)(usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_LRC);

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = (uint8_t *) &ucASCIIBuf[MB_SER_PDU_PDU_OFF];
    } else {
        eStatus = MB_EIO;
    }
    EXIT_CRITICAL_SECTION();
    return eStatus;
}

eMBErrorCode
eMBASCIISend(uint8_t ucSlaveAddress, const uint8_t *pucFrame, uint16_t usLength)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    uint8_t         usLRC;

    ENTER_CRITICAL_SECTION();
    /* Check if the receiver is still in idle state. If not, we were too
     * slow with processing the received frame and the master sent another
     * frame on the network. We have to abort sending the frame.
     */
    if (eRcvState == STATE_RX_IDLE) {
        /* First byte before the Modbus-PDU is the slave address. */
        pucSndBufferCur  = (uint8_t *) pucFrame - 1;
        usSndBufferCount = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        usSndBufferCount += usLength;

        /* Calculate LRC checksum for Modbus-Serial-Line-PDU. */
        usLRC = prvucMBLRC((uint8_t *) pucSndBufferCur, usSndBufferCount);
        ucASCIIBuf[usSndBufferCount++] = usLRC;

        /* Activate the transmitter. */
        eSndState = STATE_TX_START;
        vMBPortSerialEnable(FALSE, TRUE);
    } else {
        eStatus = MB_EIO;
    }
    EXIT_CRITICAL_SECTION();
    return eStatus;
}

BOOL
xMBASCIIReceiveFSM(void)
{
    uint8_t         ucByte;
    uint8_t         ucResult;

    // this function runs only when transmitter is idle.
    assert(eSndState == STATE_TX_IDLE);

    /* Always read the character. */
    (void)xMBPortSerialGetByte((int8_t *) &ucByte);

    // Check START_DELIMITER first.
    if (ucByte == ':') {
        /* Enable timer for character timeout. */
        vMBPortTimersEnable();
        /* Reset the input buffers to store the frame. */
        usRcvBufferPos = 0;
        eBytePos = BYTE_HIGH_NIBBLE;
        eRcvState = STATE_RX_RCV;

        return FALSE;
    }

    switch (eRcvState) {
        /* A new character is received. If the character is a ':' the input
         * buffer is cleared. A CR-character signals the end of the data
         * block. Other characters are part of the data block and their
         * ASCII value is converted back to a binary representation.
         */
    case STATE_RX_RCV:
        /* Enable timer for character timeout. */
        vMBPortTimersEnable();
        if (ucByte == MB_ASCII_DEFAULT_CR) {
            eRcvState = STATE_RX_WAIT_EOF;
            return FALSE;
        }
        ucResult = prvucMBCHAR2BIN(ucByte);
        // Prevent form processing illeagle characters.
        if (ucResult == 0xFF) {
            eRcvState = STATE_RX_IDLE;
            vMBPortTimersDisable();
            return FALSE;
        }
        if (eBytePos == BYTE_HIGH_NIBBLE) {
            /* High nibble of the byte comes first. We check for
             * a buffer overflow here. */
            if (usRcvBufferPos < MB_SER_PDU_SIZE_MAX) {
                ucASCIIBuf[usRcvBufferPos] = (uint8_t)(ucResult << 4);
                eBytePos = BYTE_LOW_NIBBLE;
            } else {
                /* not handled in Modbus specification but seems
                 * a resonable implementation. */
                eRcvState = STATE_RX_IDLE;
                /* Disable previously activated timer because of error state. */
                vMBPortTimersDisable();
            }
        } else {
            ucASCIIBuf[usRcvBufferPos] |= ucResult;
            usRcvBufferPos++;
            eBytePos = BYTE_HIGH_NIBBLE;
        }
        break;

    case STATE_RX_WAIT_EOF:
        /* Character timeout timer should be stoped, and change to STATE_RX_IDLE
         * no matter what character is received.
         * Because it should be the last character.
         */
        vMBPortTimersDisable();
        eRcvState = STATE_RX_IDLE;

        /* Post event if a correct end of frame is received. */
        if (ucByte == ucMBLFCharacter) {

            /* Notify the caller of eMBASCIIReceive that a new frame
             * was received. */
            return xMBPortEventPost(EV_FRAME_RECEIVED);
        }
        break;

    case STATE_RX_IDLE:
        break;
    }

    return FALSE;
}

BOOL
xMBASCIITransmitFSM(void)
{
    BOOL            xNeedPoll = FALSE;

    // this function runs only when receiver is idle.
    assert(eRcvState == STATE_RX_IDLE);

    switch (eSndState) {
        /* Start of transmission. The start of a frame is defined by sending
         * the character ':'. */
    case STATE_TX_START:
        xMBPortSerialPutByte((int8_t)':');
        eSndState = STATE_TX_DATA;
        eBytePos = BYTE_HIGH_NIBBLE;
        break;

        /* Send the data block. Each data byte is encoded as a character hex
         * stream with the high nibble sent first and the low nibble sent
         * last. If all data bytes are exhausted we send a '\r' character
         * to end the transmission. */
    case STATE_TX_DATA:
        if (usSndBufferCount > 0) {
            switch (eBytePos) {
            case BYTE_HIGH_NIBBLE:
                xMBPortSerialPutByte((int8_t)prvucMBBIN2CHAR((uint8_t)*pucSndBufferCur >> 4));
                eBytePos = BYTE_LOW_NIBBLE;
                break;

            case BYTE_LOW_NIBBLE:
                xMBPortSerialPutByte((int8_t)prvucMBBIN2CHAR((uint8_t)*pucSndBufferCur));
                pucSndBufferCur++;
                eBytePos = BYTE_HIGH_NIBBLE;
                usSndBufferCount--;
                break;
            }
        } else {
            xMBPortSerialPutByte(MB_ASCII_DEFAULT_CR);
            eSndState = STATE_TX_END;
        }
        break;

        /* Finish the frame by sending a LF character. */
    case STATE_TX_END:
        xMBPortSerialPutByte((int8_t)ucMBLFCharacter);
        /* We need another state to make sure that the CR character has
         * been sent. */
        eSndState = STATE_TX_NOTIFY;
        break;

        /* Notify the task which called eMBASCIISend that the frame has
         * been sent. */
    case STATE_TX_NOTIFY:
        eSndState = STATE_TX_IDLE;
        xNeedPoll = xMBPortEventPost(EV_FRAME_SENT);

        /* Disable transmitter. This prevents another transmit buffer
         * empty interrupt. */
        vMBPortSerialEnable(TRUE, FALSE);
        eSndState = STATE_TX_IDLE;
        break;

        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case STATE_TX_IDLE:
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable(TRUE, FALSE);
        break;
    }

    return xNeedPoll;
}

BOOL
xMBASCIITimerT1SExpired(void)
{
    switch (eRcvState) {
        /* If we have a timeout we go back to the idle state and wait for
         * the next frame.
         */
    case STATE_RX_RCV:
    case STATE_RX_WAIT_EOF:
        eRcvState = STATE_RX_IDLE;
        break;

    default:
        assert((eRcvState == STATE_RX_RCV) || (eRcvState == STATE_RX_WAIT_EOF));
        break;
    }
    vMBPortTimersDisable();

    /* no context switch required. */
    return FALSE;
}


static uint8_t
prvucMBCHAR2BIN(uint8_t ucCharacter)
{
    if ((ucCharacter >= '0') && (ucCharacter <= '9')) {
        return (uint8_t)(ucCharacter - '0');
    } else if ((ucCharacter >= 'A') && (ucCharacter <= 'F')) {
        return (uint8_t)(ucCharacter - 'A' + 0x0A);
    } else {
        return 0xFF;
    }
}

static uint8_t
prvucMBBIN2CHAR(uint8_t ucByte)
{
    ucByte &= 0x0F;
    if (ucByte <= 0x09) {
        return (uint8_t)('0' + ucByte);
    }
    return (uint8_t)(ucByte - 0x0A + 'A');
}


static uint8_t
prvucMBLRC(uint8_t *pucFrame, uint16_t usLen)
{
    uint8_t         ucLRC = 0;  /* LRC char initialized */

    while (usLen--) {
        ucLRC += *pucFrame++;   /* Add buffer byte without carry */
    }

    /* Return twos complement */
    ucLRC = (uint8_t)(-((int8_t) ucLRC));
    return ucLRC;
}

#endif
