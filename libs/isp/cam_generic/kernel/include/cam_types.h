/*
 * Copyright (c) 2016 Freescale Semiconductor
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
* \file     cam_types.h
* \brief    types declarations for Generic camera driver
* \author   Tomas Babinec
* \version  0.1
* \date     13.6.2016
* \note
****************************************************************************/
#ifndef CAMTYPES_H
#define CAMTYPES_H

#include "../../../sequencer/kernel/include/vdlist.h"
#include "../../../csi/kernel/include/csi_types.h"
#include "../../../viu/user/include/viu_types.h"

#ifdef __cplusplus 
extern "C" { 
#endif

#if !defined(__KERNEL__) && !defined(__STANDALONE__)
  #include <stdint.h>
#endif // if !defined(__KERNEL__) && !defined(__STANDALONE__)

#if defined(__KERNEL__) || defined(__STANDALONE__)
  #include "s32vs234.h"
#endif // #if defined(__KERNEL__) || defined(__STANDALONE__)
  
//*****************************************************************************
// const
//***************************************************************************** 
#define MAX_COMPATIBLE_BYTES 16
#define CAM_I2C_SINGLE_DATA_SZ  (sizeof(uintptr_t))
#define CAM_I2C_SINGLE_DATA_MAX (256) 
  
//*****************************************************************************
// macros
//*****************************************************************************                       
  
//*****************************************************************************
// types
//*****************************************************************************  

///< new name for SIUL2 register structure
typedef volatile struct SIUL2_tag CAM_Siul2Regs_t;

//*****************************************************************************

enum I2C_BUS_IDX
{
  I2C_BUS_IDX_0 = 0,
  I2C_BUS_IDX_1 = 1,
  I2C_BUS_IDX_2 = 2,
  I2C_BUS_IDX_INVALID,
}; // I2C_BUS_IDX index definition 

//*****************************************************************************

enum I2C_REG_ADDR_WIDTH
{
  I2C_REG_ADDR_WIDTH_8  = 0,
  I2C_REG_ADDR_WIDTH_16 = 1,
  I2C_REG_ADDR_WIDTH_MAX,
}; // I2C_REG_ADDR_WIDTH definition 

//*****************************************************************************

enum GPIO_VALUE
{
  GPIO_CLEAR  = 0,
  GPIO_SET    = 1,
}; // GPIO_VALUE definition

//*****************************************************************************

enum GPIO_PIN_CTRL_NUM
{
  GPIO_NUM_30  = 30,
  GPIO_NUM_31  = 31,
  GPIO_NUM_37  = 37,
  GPIO_NUM_38  = 38,
}; // GPIO_PIN_CTRL_NUM definition

/*****************************************************************************
* struct definitions
*****************************************************************************/

typedef struct CamI2cCfg
{
  enum I2C_BUS_IDX        mI2cBusIdx; ///< index of the I2C BUS
  uint8_t                 mI2cAddr;   ///< client I2C address
  enum I2C_REG_ADDR_WIDTH mRegAddrWidth; ///< if 0, 8bit addressing expected
  uint8_t                 mpCompatible[MAX_COMPATIBLE_BYTES];
} CamI2cCfg_t;

//*****************************************************************************

typedef struct CamGeneric
{
  VDList_Node_t mVDListNode; ///< vdlist node
  uint32_t      mIdx;        ///< index among camera clients
  CamI2cCfg_t   mCfg;        ///< config structure
#ifdef __KERNEL__  
  struct i2c_client *mI2cClient;///< I2c client structure
#endif // ifdef __KERNEL__
} CamGeneric_t;
  
//*****************************************************************************

typedef CamGeneric_t * CamI2cClient_t; // uintptr_t camera i2c client handle

//*****************************************************************************
/** Single i2c transfer description 
 *  
 * Read or Write direction defined by API call (func or IOCTL name).
 * Data are considered to contain register address + actual data values.
 * Data up to sizeof(uinptr_t) to be passed directly as value. Longer I2C 
 * message need a pointer to uint8_t array.
 *
 */

typedef struct I2cXfrSingle
{
  CamI2cClient_t mDestDev; ///< device being addressed
  uint8_t        mByteCnt; ///< number of data bytes
  uintptr_t      mData;    ///< data to be written (reg address + data) 
} I2cXfrSingle_t;
  

typedef struct I2cXfrWriteCmd
{
  uint8_t        mByteCnt; ///< number of data bytes
  uintptr_t      mData;    ///< data to be written (reg address + data) 
  uintptr_t mMask;   ///< bits not to be touched set to 1
  /* TODO: uint8_t  mByteCnt;///< number of data bytes per message */
  uint16_t mDelay;  ///< [miliseconds]; 0 = no delay introduced
  uint8_t  mReadBackCnt; ///< number of repeated checks; 0 = no check
} I2cXfrWriteCmd_t;

typedef struct I2cXfrWriteBatch
{ 
  CamI2cClient_t mDestDev; ///< device being addressed
  uint32_t       mCmdCnt;  ///< number of commands in the batch
  I2cXfrWriteCmd_t *mpCmdArr; ///< array of commands
}I2cXfrWriteBatch_t; 


typedef struct I2cXfrReadBatch
{
  CamI2cClient_t mDestDev; ///< device being addressed
  uint32_t       mCmdCnt;  ///< number of commands in the batch
  uint8_t        *mpByteCnt; ///< array of number of data bytes
  uintptr_t      *mpData;    ///< array of data to be written (reg address + data) 
} I2cXfrReadBatch_t;


typedef struct GpioControlCmd
{
  enum GPIO_PIN_CTRL_NUM    mGpioPin; ///< GPIO pin number to control
  enum GPIO_VALUE           mVal;     ///< value to Set/Clear pin output
} GpioControlCmd_t;

//*****************************************************************************

#if defined(__KERNEL__) || defined(__STANDALONE__)

typedef struct mutex mutex_t;

#endif // #if defined(__KERNEL__) || defined(__STANDALONE__)

#ifdef __cplusplus 
} //extern "C"
#endif

#endif /* CAMTYPES_H */