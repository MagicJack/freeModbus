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
 * File: $Id: mb.c,v 1.28 2010/06/06 13:54:40 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbfunc.h"

#include "mbport.h"
#if MB_RTU_ENABLED == 1
#include "mbrtu.h"
#endif
#if MB_ASCII_ENABLED == 1
#include "mbascii.h"
#endif
#if MB_TCP_ENABLED == 1
#include "mbtcp.h"
#endif

#ifndef MB_PORT_HAS_CLOSE
#define MB_PORT_HAS_CLOSE 0
#endif

/* ----------------------- Static variables ---------------------------------*/

static uint8_t  ucMBAddress;
static eMBMode  eMBCurrentMode;

static enum
{
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
} eMBState = STATE_NOT_INITIALIZED;

/* Functions pointer which are initialized in eMBInit(). Depending on the
 * mode (RTU or ASCII) the are set to the correct implementations.
 */
static peMBFrameSend    peMBFrameSendCur;
static pvMBFrameStart   pvMBFrameStartCur;
static pvMBFrameStop    pvMBFrameStopCur;
static peMBFrameReceive peMBFrameReceiveCur;
static pvMBFrameClose   pvMBFrameCloseCur;

/* Callback functions required by the porting layer. They are called when
 * an external event has happend which includes a timeout or the reception
 * or transmission of a character.
 */
BOOL    (*pxMBFrameCBByteReceived)(void);
BOOL    (*pxMBFrameCBTransmitterEmpty)(void);
BOOL    (*pxMBPortCBTimerExpired)(void);

/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 */
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX] = {
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    { MB_FUNC_OTHER_REPORT_SLAVEID, eMBFuncReportSlaveID },
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    { MB_FUNC_READ_INPUT_REGISTER, eMBFuncReadInputRegister },
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    { MB_FUNC_READ_HOLDING_REGISTER, eMBFuncReadHoldingRegister },
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    { MB_FUNC_WRITE_MULTIPLE_REGISTERS, eMBFuncWriteMultipleHoldingRegister },
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    { MB_FUNC_WRITE_REGISTER, eMBFuncWriteHoldingRegister },
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    { MB_FUNC_READWRITE_MULTIPLE_REGISTERS, eMBFuncReadWriteMultipleHoldingRegister },
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    { MB_FUNC_READ_COILS, eMBFuncReadCoils },
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    { MB_FUNC_WRITE_SINGLE_COIL, eMBFuncWriteCoil },
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    { MB_FUNC_WRITE_MULTIPLE_COILS, eMBFuncWriteMultipleCoils },
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    { MB_FUNC_READ_DISCRETE_INPUTS, eMBFuncReadDiscreteInputs },
#endif
};

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBInit(eMBMode eMode, uint8_t ucSlaveAddress, uint8_t ucPort, uint32_t ulBaudRate, eMBParity eParity)
{
    eMBErrorCode    eStatus = MB_ENOERR;

    /* check preconditions */
    if ((ucSlaveAddress == MB_ADDRESS_BROADCAST) ||
        (ucSlaveAddress < MB_ADDRESS_MIN) ||
        (ucSlaveAddress > MB_ADDRESS_MAX))
        return MB_EINVAL;

    ucMBAddress = ucSlaveAddress;

    switch (eMode) {
#if MB_RTU_ENABLED > 0
    case MB_RTU:
        pvMBFrameStartCur           = eMBRTUStart;
        pvMBFrameStopCur            = eMBRTUStop;
        peMBFrameSendCur            = eMBRTUSend;
        peMBFrameReceiveCur         = eMBRTUReceive;
        pvMBFrameCloseCur           = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
        pxMBFrameCBByteReceived     = xMBRTUReceiveFSM;
        pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM;
        pxMBPortCBTimerExpired      = xMBRTUTimerT35Expired;

        eStatus = eMBRTUInit(ucMBAddress, ucPort, ulBaudRate, eParity);
        break;
#endif
#if MB_ASCII_ENABLED > 0
    case MB_ASCII:
        pvMBFrameStartCur           = eMBASCIIStart;
        pvMBFrameStopCur            = eMBASCIIStop;
        peMBFrameSendCur            = eMBASCIISend;
        peMBFrameReceiveCur         = eMBASCIIReceive;
        pvMBFrameCloseCur           = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
        pxMBFrameCBByteReceived     = xMBASCIIReceiveFSM;
        pxMBFrameCBTransmitterEmpty = xMBASCIITransmitFSM;
        pxMBPortCBTimerExpired      = xMBASCIITimerT1SExpired;

        eStatus = eMBASCIIInit(ucMBAddress, ucPort, ulBaudRate, eParity);
        break;
#endif
    default:
        eStatus = MB_EINVAL;
    }

    if (eStatus == MB_ENOERR) {
        if (!xMBPortEventInit()) {
            /* port dependent event module initalization failed. */
            eStatus = MB_EPORTERR;
        } else {
            eMBCurrentMode = eMode;
            eMBState = STATE_DISABLED;
        }
    }
    return eStatus;
}

#if MB_TCP_ENABLED > 0
eMBErrorCode
eMBTCPInit(uint16_t ucTCPPort)
{
    if (eMBTCPDoInit(ucTCPPort) != MB_ENOERR) {
        eMBState = STATE_DISABLED;
        return MB_ENOERR;
    }

    /* Port dependent event module initalization failed. */
    if (!xMBPortEventInit())
        return MB_EPORTERR;

    pvMBFrameStartCur   = eMBTCPStart;
    pvMBFrameStopCur    = eMBTCPStop;
    peMBFrameReceiveCur = eMBTCPReceive;
    peMBFrameSendCur    = eMBTCPSend;
    pvMBFrameCloseCur   = MB_PORT_HAS_CLOSE ? vMBTCPPortClose : NULL;
    ucMBAddress         = MB_TCP_PSEUDO_ADDRESS;
    eMBCurrentMode      = MB_TCP;

    eMBState = STATE_DISABLED;
    return MB_ENOERR;
}
#endif

eMBErrorCode
eMBRegisterCB(uint8_t ucFunctionCode, pxMBFunctionHandler pxHandler)
{
    int             i;

    if ((ucFunctionCode > 127))
        return MB_EINVAL;

    if (pxHandler != NULL) {
        for (i = 0; i < MB_FUNC_HANDLERS_MAX; i++) {
            if ((xFuncHandlers[i].pxHandler == NULL) ||
                (xFuncHandlers[i].pxHandler == pxHandler)) {
                ENTER_CRITICAL_SECTION();
                xFuncHandlers[i].ucFunctionCode = ucFunctionCode;
                xFuncHandlers[i].pxHandler = pxHandler;
                EXIT_CRITICAL_SECTION();
                return MB_ENOERR;
            }
        }
        return MB_ENORES;
    } else {
        for (i = 0; i < MB_FUNC_HANDLERS_MAX; i++) {
            if (xFuncHandlers[i].ucFunctionCode == ucFunctionCode) {
                ENTER_CRITICAL_SECTION();
                xFuncHandlers[i].ucFunctionCode = 0;
                xFuncHandlers[i].pxHandler = NULL;
                EXIT_CRITICAL_SECTION();
                break;
            }
        }
        /* Remove can't fail. */
        return MB_ENOERR;
    }
}


eMBErrorCode
eMBClose(void)
{
    if (eMBState != STATE_DISABLED)
        return MB_EILLSTATE;

    if (pvMBFrameCloseCur != NULL)
        pvMBFrameCloseCur();

    return MB_ENOERR;
}

eMBErrorCode
eMBEnable(void)
{
    if (eMBState != STATE_DISABLED)
        return MB_EILLSTATE;

    /* Activate the protocol stack. */
    pvMBFrameStartCur();
    eMBState = STATE_ENABLED;

    return MB_ENOERR;
}

eMBErrorCode
eMBDisable(void)
{
    if (eMBState != STATE_ENABLED) {
        if (eMBState == STATE_DISABLED)
            return MB_ENOERR;
        else
            return MB_EILLSTATE;
    }

    pvMBFrameStopCur();
    eMBState = STATE_DISABLED;

    return MB_ENOERR;
}

eMBErrorCode
eMBPoll(void)
{
    static uint8_t     *ucMBFrame;
    static uint8_t      ucRcvAddress;
    static uint8_t      ucFunctionCode;
    static uint16_t     usLength;
    static eMBException eException;

    int             i;
    eMBErrorCode    eStatus = MB_ENOERR;
    eMBEventType    eEvent;

    /* Check if the protocol stack is ready. */
    if (eMBState != STATE_ENABLED)
        return MB_EILLSTATE;

    /* Check if there is a event available. If not return control to caller.
     * Otherwise we will handle the event. */
    if (!xMBPortEventGet(&eEvent))
        return MB_ENOERR;

    switch (eEvent) {
    case EV_READY:
        break;

    case EV_FRAME_RECEIVED:
        eStatus = peMBFrameReceiveCur(&ucRcvAddress, &ucMBFrame, &usLength);
        if (eStatus == MB_ENOERR) {
            /* Check if the frame is for us. If not ignore the frame. */
            if ((ucRcvAddress == ucMBAddress) || (ucRcvAddress == MB_ADDRESS_BROADCAST)) {
                (void)xMBPortEventPost(EV_EXECUTE);
            }
        }
        break;

    case EV_EXECUTE:
        ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];
        eException = MB_EX_ILLEGAL_FUNCTION;
        for (i = 0; i < MB_FUNC_HANDLERS_MAX; i++) {
            /* No more function handlers registered. Abort. */
            if (xFuncHandlers[i].ucFunctionCode == 0) {
                break;
            } else if (xFuncHandlers[i].ucFunctionCode == ucFunctionCode) {
                eException = xFuncHandlers[i].pxHandler(ucMBFrame, &usLength);
                break;
            }
        }

        /* If the request was not sent to the broadcast address we
         * return a reply. */
        if (ucRcvAddress != MB_ADDRESS_BROADCAST) {
            if (eException != MB_EX_NONE) {
                /* An exception occured. Build an error frame. */
                usLength = 0;
                ucMBFrame[usLength++] = (uint8_t)(ucFunctionCode | MB_FUNC_ERROR);
                ucMBFrame[usLength++] = eException;
            }
            if ((eMBCurrentMode == MB_ASCII) && MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS) {
                vMBPortTimersDelay(MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS);
            }
            eStatus = peMBFrameSendCur(ucMBAddress, ucMBFrame, usLength);
        }
        break;

    case EV_FRAME_SENT:
        break;
    }
    return MB_ENOERR;
}
