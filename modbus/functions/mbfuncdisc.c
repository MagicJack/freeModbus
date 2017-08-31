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
 * File: $Id: mbfuncdisc.c,v 1.8 2007/02/18 23:47:16 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbconfig.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_FUNC_READ_ADDR_OFF           (MB_PDU_DATA_OFF)
#define MB_PDU_FUNC_READ_DISCCNT_OFF        (MB_PDU_DATA_OFF + 2)
#define MB_PDU_FUNC_READ_SIZE               (4)
#define MB_PDU_FUNC_READ_DISCCNT_MAX        (0x07D0)

/* ----------------------- Static functions ---------------------------------*/
eMBException    prveMBError2Exception(eMBErrorCode eErrorCode);

/* ----------------------- Start implementation -----------------------------*/
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0

eMBException
eMBFuncReadDiscreteInputs(uint8_t *pucFrame, uint16_t *usLen)
{
    uint16_t        usRegAddress;
    uint16_t        usDiscreteCnt;
    uint8_t         ucNBytes;
    uint8_t        *pucFrameCur;
    eMBErrorCode    eRegStatus;

    /* Can't be a valid read coil register request because the length
     * is incorrect. */
    if (*usLen != (MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN))
        return MB_EX_ILLEGAL_DATA_VALUE;

    usRegAddress  = (uint16_t)(pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8);
    usRegAddress |= (uint16_t)(pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1]);
    usRegAddress++;

    usDiscreteCnt  = (uint16_t)(pucFrame[MB_PDU_FUNC_READ_DISCCNT_OFF] << 8);
    usDiscreteCnt |= (uint16_t)(pucFrame[MB_PDU_FUNC_READ_DISCCNT_OFF + 1]);

    /* Check if the number of registers to read is valid. If not
     * return Modbus illegal data value exception.
     */
    if ((usDiscreteCnt == 0) ||
        (usDiscreteCnt > MB_PDU_FUNC_READ_DISCCNT_MAX))
        return MB_EX_ILLEGAL_DATA_VALUE;


    /* Set the current PDU data pointer to the beginning. */
    pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
    *usLen = MB_PDU_FUNC_OFF;

    /* First byte contains the function code. */
    *pucFrameCur++ = MB_FUNC_READ_DISCRETE_INPUTS;
    *usLen += 1;

    /* Test if the quantity of coils is a multiple of 8. If not last
     * byte is only partially field with unused coils set to zero.
     */
    ucNBytes = (uint8_t)(usDiscreteCnt/8) + (usDiscreteCnt%8 != 0);

    *pucFrameCur++ = ucNBytes;
    *usLen += 1;

    eRegStatus =
        eMBRegDiscreteCB(pucFrameCur, usRegAddress, usDiscreteCnt);

    /* If an error occured convert it into a Modbus exception. */
    if (eRegStatus != MB_ENOERR)
        return prveMBError2Exception(eRegStatus);

    /* The response contains the function code, the starting address
     * and the quantity of registers. We reuse the old values in the
     * buffer because they are still valid. */
    *usLen += ucNBytes;
     return MB_EX_NONE;
}

#endif
