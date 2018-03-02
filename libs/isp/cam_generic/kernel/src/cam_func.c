/*
 * Copyright (c) 2016 Freescale Semiconductor
 * Copyright 2016-2017 NXP
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
 * \file    csi_func.c
 * \brief   Mipi csi receiver interface functionality implementation.
 * \author  Tomas Babinec
 * \version 0.1
 * \date    18-September-2015
 ****************************************************************************/

#if !defined(__STANDALONE__)
  #include <linux/kernel.h>     // We're doing kernel work
  #include <linux/module.h>     // Specifically, a module 
  #include <linux/interrupt.h>  // For interrupts
  #include <asm/siginfo.h>      // siginfo
  #include <linux/sched.h>      // send_sig_info
  #include <linux/types.h>
  #include <linux/delay.h>      // for msleep
  #include <linux/slab.h>       // for malloc and free
  #include <linux/i2c.h>
  #include <asm/io.h>           // for ioremap
  #include <linux/gpio.h>       // Required for the GPIO functions
  #include <linux/of_platform.h>
  #include <linux/platform_device.h>
#else 
  #include <stdio.h>
  #include <unistd.h>
  #include <stdint.h>
  #include <stddef.h>
  #include <math.h>
  #include "i2c.h"
  
  #include "seq_sa_kernel_api.h"
  

  unsigned long get_uptime_microS(void);

  #define msleep(_val) \
        {\
          unsigned long lStart = get_uptime_microS();\
          while((get_uptime_microS() - lStart) < _val*1000);\
        }  
#endif // else from if !defined(__STANDALONE__)

#include "config.h"
#include "cam.h"
#include "csi_func.h"
#include "cam_func.h"
#include "vdb_log.h"
#if defined(__KERNEL__)
  #include <linux/i2c.h>
#endif // #ifdef __KERNEL__

//****************************************************************************
// consts
//****************************************************************************
#define I2C_CLI_CNT_MAX 50 // number of supported I2C clients

//****************************************************************************
// macros
//****************************************************************************

//****************************************************************************
// static global variables
//***************************************************************************/
static uint32_t sCliAct = 0; ///< number of active i2c clients

static VDList_t sCamList;    ///< list of active cameras

static uint32_t sI2cRepeat = 0; ///< the number of I2C retransimission's repeat

#ifdef __KERNEL__

static int CamI2c_Probe(struct i2c_client *client,
                          const struct i2c_device_id *id);
static int CamI2c_Remove(struct i2c_client *client);

static const struct i2c_device_id sCamId[] = 
{
  {"s32v-cam-generic", 0},
  {},
};

static struct i2c_driver sCamI2cDriver = 
{
  .driver = 
  {
    .owner  = THIS_MODULE,
    .name   = "s32v-cam-generic",
  },
  .probe    = CamI2c_Probe,
  .remove   = CamI2c_Remove,
  .id_table = sCamId,
};

//****************************************************************************
// global variables
//****************************************************************************

//****************************************************************************
// functions declarations
//****************************************************************************

/****************************************************************************/
/** Remove specified camera client.
 * 
 * \param apCfg camera client config
 * 
 * \return pointer to i2c client structure if all ok,
 *         NULL otherwise.
 ****************************************************************************/
static struct i2c_client* CAM_DRV_I2cClientAdd(const CamI2cCfg_t *const apCfg);

/****************************************************************************/
/** Remove specified camera client.
 * 
 * \param apClient i2c client config
 * 
 * \return CAM_DRV_SUCCESS if all ok,
 *         CAM_DRV_FAILURE otherwise.
 ****************************************************************************/
static int32_t CAM_DRV_I2cClientRem(struct i2c_client *const apClient);

/****************************************************************************/
/** Invokes I2c write transaction.
 *  
 * \param apcXfr   I2C transfer definition
 * 
 * \return CAM_DRV_SUCCESS if all ok
 *         CAM_DRV_FAILURE otherwise
 ****************************************************************************/
static int32_t LinuxI2cWrite(const I2cXfrSingle_t *apcXfr);

/****************************************************************************/
/** Invokes I2c read transaction.
 *  
 * \param apcXfr   I2C transfer definition
 * 
 * \return CAM_DRV_SUCCESS if all ok
 *         CAM_DRV_FAILURE otherwise
 ****************************************************************************/
static int32_t LinuxI2cRead(I2cXfrSingle_t *apXfr);

#endif // ifdef __KERNEL__

//****************************************************************************
// functions definitions
//****************************************************************************

int32_t CAM_DRV_Setup()
{ 
  int32_t lRet = CAM_DRV_SUCCESS;
  
  VDList_Init(&sCamList);
  
#ifdef __STANDALONE__  
  I2C_Init_Config(I2C0, I2C_SONY_CAM_ADDR << 1);  
  I2C_Init_Config(I2C1, I2C_SONY_CAM_ADDR << 1);
  I2C_Init_Config(I2C2, I2C_SONY_CAM_ADDR << 1);
#endif //#ifdef __STANDALONE__

#ifdef __KERNEL__
   gpio_request(GPIO_NUM_30, CAM_GPIO_GPIO30_LABEL);   // gpio30
   gpio_request(GPIO_NUM_31, CAM_GPIO_GPIO31_LABEL);   // gpio31
   gpio_request(GPIO_NUM_37, CAM_GPIO_GPIO37_LABEL);   // gpio37
   gpio_request(GPIO_NUM_38, CAM_GPIO_GPIO38_LABEL);   // gpio38

   gpio_direction_output(GPIO_NUM_30, 1);   // Set the gpio to be in output mode and turn on
   gpio_direction_output(GPIO_NUM_31, 1);   // Set the gpio to be in output mode and turn on
   gpio_direction_output(GPIO_NUM_37, 1);   // Set the gpio to be in output mode and turn on
   gpio_direction_output(GPIO_NUM_38, 1);   // Set the gpio to be in output mode and turn on

   gpio_export(GPIO_NUM_30, false);  // causes gpio to appear in /sys/class/gpio
   gpio_export(GPIO_NUM_31, false);  // causes gpio to appear in /sys/class/gpio
   gpio_export(GPIO_NUM_37, false);  // causes gpio to appear in /sys/class/gpio
   gpio_export(GPIO_NUM_38, false);  // causes gpio to appear in /sys/class/gpio
                                 // the second argument prevents the direction from being changed
   gpio_set_value(GPIO_NUM_30, 1);
   gpio_set_value(GPIO_NUM_31, 1);
   gpio_set_value(GPIO_NUM_37, 1);
   gpio_set_value(GPIO_NUM_38, 1);
#endif // #ifdef __KERNEL__

  return lRet;
} // CAM_DRV_Setup()

//****************************************************************************

int32_t CAM_DRV_Close()
{
  int32_t lRet = CAM_DRV_SUCCESS;
    
  /// *** remove all active clients ***
  while((int32_t)1 != VDList_Empty(&sCamList))
  {
    const CamI2cClient_t lCamHandle = (CamI2cClient_t) sCamList.mpTail->mpData;
    if(lCamHandle != 0)
    {
      if(CAM_DRV_ClientRem(lCamHandle) != CAM_DRV_SUCCESS) 
      {
        VDB_LOG_ERROR("Failed to remove camera client\n");      
        lRet = CAM_DRV_FAILURE;
      } // if client remove failed
    } // if cam is NULL
    else
    {
      VDB_LOG_ERROR("Camera pointer is NULL.\n");
      lRet = CAM_DRV_FAILURE;
    } // else from if cam is NULL

    if(I2C_BUS_IDX_0 == lCamHandle->mCfg.mI2cBusIdx)
    {
      CSI_IrqHandlerRegister(CSI_IDX_0, NULL);
      VDB_LOG_NOTE("Unregister CSI Irq handler for CSI 0\n");
    }
    else if(I2C_BUS_IDX_1 == lCamHandle->mCfg.mI2cBusIdx)
    {
      CSI_IrqHandlerRegister(CSI_IDX_1, NULL);
      VDB_LOG_NOTE("Unregister CSI Irq handler for CSI 1\n");
    }
  } // while not empty

#ifdef __KERNEL__
   // set all output gpio pins to low value
   gpio_set_value(GPIO_NUM_30, 0);
   gpio_set_value(GPIO_NUM_31, 0);
   gpio_set_value(GPIO_NUM_37, 0);
   gpio_set_value(GPIO_NUM_38, 0);

  // unexport all output gpio pins
   gpio_unexport(GPIO_NUM_30);
   gpio_unexport(GPIO_NUM_31);
   gpio_unexport(GPIO_NUM_37);
   gpio_unexport(GPIO_NUM_38);

   // free all gpio pins
   gpio_free(GPIO_NUM_30);
   gpio_free(GPIO_NUM_31);
   gpio_free(GPIO_NUM_37);
   gpio_free(GPIO_NUM_38);
#endif // #ifdef __KERNEL__

  return lRet;
} //CAM_DRV_Close()

//****************************************************************************
// Client management
//****************************************************************************

CamI2cClient_t CAM_DRV_ClientAdd(const CamI2cCfg_t * const apCfg)
{
  CamI2cClient_t lRet  = 0;
  CamGeneric_t  *lpCam = NULL;
  
  lpCam = (CamGeneric_t *)kmalloc(sizeof(CamGeneric_t), GFP_KERNEL);
  
  if(lpCam != NULL)
  {
    lpCam->mVDListNode.mpData = (void *)lpCam;
#ifdef __KERNEL__
    lpCam->mI2cClient = CAM_DRV_I2cClientAdd(apCfg);
    if(NULL == lpCam->mI2cClient)
    {
      VDB_LOG_ERROR(
        "Failed to create I2C client (I2C addr 0x%x.)\n",
        lpCam->mCfg.mI2cAddr
      );
      
      kfree(lpCam);
    } // if failed to create I2C client
    else
    {
#endif // #ifdef __KERNEL__
      VDB_LOG_NOTE("CGM: Client added (I2C addr 0x%x)\n", lpCam->mCfg.mI2cAddr);
      lpCam->mCfg = *apCfg;    
      lRet = (CamI2cClient_t)lpCam;

      VDList_HeadPush(&sCamList, &(lpCam->mVDListNode));
#ifdef __KERNEL__
    }
#endif // #ifdef __KERNEL__
  } // if allocation succeeded
  else
  {
    VDB_LOG_ERROR("Failed to allocate memory for camera \n");
  } // else from if allocation succeeded

  return lRet;
} // CAM_DRV_ClientAdd()

//*****************************************************************************

int32_t CAM_DRV_ClientRem(const CamI2cClient_t aClient)
{
  int32_t       lRet  = CAM_DRV_SUCCESS;
  CamGeneric_t *const lpCam = aClient;
  
  if( lpCam != NULL)
  {
#ifdef __KERNEL__
    lRet = CAM_DRV_I2cClientRem(lpCam->mI2cClient);
#endif // #ifdef __KERNEL__
    VDB_LOG_NOTE("CGM: Client removed (I2C addr 0x%x)\n", lpCam->mCfg.mI2cAddr);
    VDList_Rem(&sCamList, &(lpCam->mVDListNode));
    kfree(lpCam);
          
  } // if Camera client exists
  else
  {
    lRet = CAM_DRV_FAILURE;
    VDB_LOG_ERROR("Bad Camera client handle.\n");
  } // else from if Camera client exists
  
  return lRet;
} //CAM_DRV_ClientRem()

//****************************************************************************
// I2c Read/Write
//****************************************************************************

int32_t CAM_DRV_Write(const I2cXfrSingle_t *apXfr)
{
  int32_t lRet = CAM_DRV_SUCCESS;
  
  // *** write new value ***
#ifdef __STANDALONE__  
  uint8_t *lpData;

  if(apXfr->mByteCnt <= CAM_I2C_SINGLE_DATA_SZ)
  {
    lpData = (uint8_t*)(&apXfr->mData);
  } // if small data
  else
  {
    lpData = (uint8_t*)(apXfr->mData);
  } // else from if small data   

  // VDB_LOG_NOT_IMPLEMENTED();
  I2CX_DevTx(apXfr->mDestDev->mCfg.mI2cBusIdx, 
             apXfr->mDestDev->mCfg.mI2cAddr, 
             lpData, 
             apXfr->mByteCnt);
#else
  lRet = LinuxI2cWrite(apXfr);//LinuxI2cWrite chua xet if(apXfr->mByteCnt <= CAM_I2C_SINGLE_DATA_SZ)
#endif // #ifdef __STANDALONE__ 
  
  return lRet;
} // CAM_DRV_RegWrite8()

//****************************************************************************

int32_t CAM_DRV_Read(I2cXfrSingle_t *apXfr)
{
  int32_t lRet = CAM_DRV_SUCCESS;
#ifdef __STANDALONE__   
  // VDB_LOG_NOT_IMPLEMENTED();
  uint8_t lAddrLen = 1;
  uint8_t *lpData  = NULL;

  if(apXfr->mByteCnt <= CAM_I2C_SINGLE_DATA_SZ)
  {
    lpData = (uint8_t*)(&apXfr->mData);
  } // if small data
  else
  {
    lpData = (uint8_t*)(apXfr->mData);
  } // else from if small data

  switch(apXfr->mDestDev->mCfg.mRegAddrWidth)
  {
    case I2C_REG_ADDR_WIDTH_8:
    {
      lAddrLen = 1;
      break;
    } // I2C_REG_ADDR_WIDTH_8

    case I2C_REG_ADDR_WIDTH_16:
    {
      lAddrLen = 2;
      break;
    } // I2C_REG_ADDR_WIDTH_16

    default:
    {
      VDB_LOG_ERROR("Not supported!\n");
      lRet = CAM_DRV_FAILURE;
      break;
    } // default
  } // switch(apXfr->mDestDev->mCfg.mRegAddrWidth)

  if(lRet == CAM_DRV_SUCCESS)
  {
    I2CX_DevRx(apXfr->mDestDev->mCfg.mI2cBusIdx, 
             apXfr->mDestDev->mCfg.mI2cAddr, 
             lpData, 
             lAddrLen,
             lpData + lAddrLen,
             apXfr->mByteCnt - lAddrLen);
  } // if(lRet == CAM_DRV_SUCCESS)

#else
  lRet = LinuxI2cRead(apXfr);
#endif // #ifdef __STANDALONE__ 
  return lRet;
} //CAM_DRV_RegRead8()

//****************************************************************************

int32_t CAM_DRV_CmdWrite(const CamGeneric_t  *const apCam, 
                  I2cXfrWriteCmd_t *const apCmd)
{
  int32_t lRet = CAM_DRV_SUCCESS;
  uint32_t i = 0;
  I2cXfrSingle_t lXfr;
  uint8_t *lpData;
  uint8_t *lpVal;
  uint8_t *lpMask;
  I2cXfrSingle_t lXfrRead;
  uint8_t lAddrLen = 1;
  uint8_t lpI2cReadData[CAM_I2C_SINGLE_DATA_MAX] = {0};

  switch(apCam->mCfg.mRegAddrWidth)
  {
    case I2C_REG_ADDR_WIDTH_8:
    {
      lAddrLen = 1;
      break;
    } // I2C_REG_ADDR_WIDTH_8

    case I2C_REG_ADDR_WIDTH_16:
    {
      lAddrLen = 2;
      break;
    } // I2C_REG_ADDR_WIDTH_16

    default:
    {
      VDB_LOG_ERROR("Not supported!\n");
      lRet = CAM_DRV_FAILURE;
      break;
    } // default
  } // switch(apCam->mCfg.mRegAddrWidth)

  //memset(&lXfr, 0, sizeof(I2cXfrSingle_t));
  lXfr.mDestDev = (CamI2cClient_t)apCam;
  lXfr.mData = apCmd->mData;
  lXfr.mByteCnt = apCmd->mByteCnt;

  //prepare for check data
  lXfrRead.mDestDev = (CamI2cClient_t)apCam;
  lXfrRead.mData = apCmd->mData;
  lXfrRead.mByteCnt = apCmd->mByteCnt;

  if (apCmd->mByteCnt > CAM_I2C_SINGLE_DATA_SZ)
  {
    //Copy address data for reading I2C data
    lpData = (uint8_t*)apCmd->mData;
    
    for (i = 0; i < lAddrLen; i++)
    {
      lpI2cReadData[i] = lpData[i];
    }

    lXfrRead.mData = (uintptr_t)lpI2cReadData;
  }// if apCmd->mByteCnt > CAM_I2C_SINGLE_DATA_SZ

  // *** if mask set read old value first ***
  if(((uintptr_t)0) != apCmd->mMask) 
  {
    
    if(CAM_DRV_Read(&lXfrRead) != CAM_DRV_SUCCESS)
    {
      VDB_LOG_ERROR("Failed to read value to apply mask.\n");
      lRet = CAM_DRV_FAILURE;
    } // if read failed
    else
    {
      //apply mask
      if (apCmd->mByteCnt > CAM_I2C_SINGLE_DATA_SZ)
      {
        lpVal = ((uint8_t*)lXfrRead.mData) + lAddrLen;
        lpData = ((uint8_t*)lXfr.mData) + lAddrLen;
        lpMask = (uint8_t*)apCmd->mMask;
      }
      else
      {
        lpVal = ((uint8_t*)&lXfrRead.mData) + lAddrLen;
        lpData = ((uint8_t*)&lXfr.mData) + lAddrLen;
        lpMask = (uint8_t*)&apCmd->mMask;
      }

      for (i = 0; i < (uint32_t)(apCmd->mByteCnt - lAddrLen); i++)
      {
        *lpData = (*lpData & (~(*lpMask))) | ((*lpVal) & (*lpMask));
        lpData++;
        lpVal++;
        lpMask++;
      }
     
    }
  
  } // if mask is used
  
  if (CAM_DRV_SUCCESS == lRet)
  {
    // *** write new value ***
    CAM_DRV_Write(&lXfr);
  
    // *** wait as requested ***
    if(((uint16_t)0) != apCmd->mDelay)
    {
      msleep(apCmd->mDelay);
    } // if delay required
  
    // *** read back and check if requested ***
    while(i < (apCmd->mReadBackCnt))
    {
      uint8_t lCheck = 0;
      lRet = CAM_DRV_Read(&lXfrRead);
      if(CAM_DRV_SUCCESS == lRet)
      {
        // check
        if (apCmd->mByteCnt > CAM_I2C_SINGLE_DATA_SZ)
        {
            //Copy data
          lpVal = ((uint8_t*)lXfrRead.mData) + lAddrLen;
          lpData = ((uint8_t*)lXfr.mData) + lAddrLen;
        }
        else
        { 
          lpVal = ((uint8_t*)&lXfrRead.mData) + lAddrLen;
          lpData = ((uint8_t*)&lXfr.mData) + lAddrLen;

        }

        for (i = 0; i < (uint32_t)(apCmd->mByteCnt - lAddrLen); i++)
        {
          if(*lpVal != *lpData)
          {
            lCheck = 1;
            break;
          } 
          lpVal++;
          lpData++;
        }

        if ( 0 == lCheck)
        {
          break;
        }

      } // if read successful
    
      // *** wait as requested ***
      msleep(apCmd->mDelay);
      i++;
    } // while not all attempts done
  } // if READ is successful  
  return lRet;
} // CAM_DRV_CmdWrite()

//****************************************************************************
// I2c Transmission Repeat
//****************************************************************************

int32_t CAM_DRV_I2C_RepeatSet(const uint32_t aI2cRepeat)
{
  int32_t lRet = CAM_DRV_SUCCESS;

  sI2cRepeat = aI2cRepeat;
  
  return lRet;
} //CAM_DRV_I2C_RepeatSet()

//*****************************************************************************

int32_t CAM_DRV_I2C_RepeatGet(uint32_t * const apI2cRepeat)
{
  int32_t lRet = CAM_DRV_SUCCESS;

  *apI2cRepeat = sI2cRepeat;
  
  return lRet;
} //CAM_DRV_I2C_RepeatSet()

//*****************************************************************************
// Linux kernel related functionality
//*****************************************************************************

#ifdef __KERNEL__
// *** I2C client Add/Rem***

static struct i2c_client* CAM_DRV_I2cClientAdd(const CamI2cCfg_t *const apCfg)
{
  struct i2c_adapter *lpI2cAdap  = NULL;
  int    lI2cBusIdx = (int)apCfg->mI2cBusIdx;
  struct i2c_client  *lpI2cCli   = NULL;
  int lRetPrint = 0;

  lRetPrint = pr_info(
    "Probing I2c client @ 0x%x on I2C bus #%u.\n", 
    apCfg->mI2cAddr,
    lI2cBusIdx);

  lpI2cAdap = i2c_get_adapter(lI2cBusIdx);
  if(lpI2cAdap != NULL)
  {
    struct i2c_board_info lpI2cInfo = 
    {
      I2C_BOARD_INFO("s32v-cam-generic", apCfg->mI2cAddr)
    };
    
    lpI2cCli = i2c_new_device(lpI2cAdap, &lpI2cInfo);
    i2c_put_adapter(lpI2cAdap);
    if(lpI2cCli == NULL)
    {
      VDB_LOG_ERROR("I2c client is NULL.\n");
    } // if client not created
    else
    { 
      if(sCliAct == (uint32_t)0)
      {
        if(0 != i2c_add_driver(&sCamI2cDriver))
        {
          VDB_LOG_ERROR("Failed to add generic camera i2c driver.\n");
          i2c_unregister_device(lpI2cCli);
          lpI2cCli = NULL;
        } // if add Sony i2c client driver failed
        else
        {
          lRetPrint = pr_info("Generic camera I2c driver added successfully.\n");
        } // else from if add Sony i2c client driver failed
      } // if driver not registerred already
      sCliAct++;
    } // else from if no i2c client
  } // if I2C addapter not NULL
  else
  {
    VDB_LOG_ERROR("Faled to get I2C adapter %u.\n", lI2cBusIdx);
  } // else from if I2C addapter not NULL
  
  return lpI2cCli;
} // CAM_DRV_I2cClientAdd()

//*****************************************************************************

static int32_t CAM_DRV_I2cClientRem(struct i2c_client *const apClient)
{
  int32_t lRet = CAM_DRV_SUCCESS;

  if(apClient != NULL)
  {
    if(sCliAct == (uint32_t)1)
    {
      i2c_del_driver(&sCamI2cDriver);
    } // if last device active

    i2c_unregister_device(apClient);
    sCliAct--;
  } // if Client exists
  
  return lRet;
} //CAM_DRV_I2cClientRem()

//****************************************************************************

static int CamI2c_Probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{ 
  return 0;
} // SonyI2c_Probe()

//*****************************************************************************

static int CamI2c_Remove(struct i2c_client *client)
{
  return 0;
}  // SonyI2c_Remove()

//*****************************************************************************
// *** I2c linux access ***

static int32_t LinuxI2cWrite(const I2cXfrSingle_t *apcXfr)
{
  int32_t            lNumBytes   = 0;
  int32_t            lRetVal     = CAM_DRV_SUCCESS;  
  struct i2c_client *lpI2cClient = apcXfr->mDestDev->mI2cClient;
  uint8_t           *lpData;
  uint32_t           lLoop = 0;

  if(apcXfr->mByteCnt <= CAM_I2C_SINGLE_DATA_SZ)
  {
    lpData = (uint8_t*)(&apcXfr->mData);
  } // if small data
  else
  {
    lpData = (uint8_t*)(apcXfr->mData);
  } // else from if small data
  
  if(lpI2cClient != NULL)
  {
    for (lLoop = sI2cRepeat ; lLoop >= 0 ; lLoop--)
    {
      lNumBytes = (int32_t)i2c_master_send(lpI2cClient, 
                                         (char *)lpData, 
                                         apcXfr->mByteCnt);
    
      if (lNumBytes != apcXfr->mByteCnt) 
      {
         if (0 == lLoop)
         {
           lRetVal = CAM_DRV_FAILURE;
           VDB_LOG_ERROR("W err:device_address=0x%x, i2c_bus_idx=%d, data=0x%x|%x|%x|%x, ret val:%d\n",
                     lpI2cClient->addr, apcXfr->mDestDev->mCfg.mI2cBusIdx,
                     lpData[0],  lpData[1], lpData[2], lpData[3],
                     lNumBytes);
           break;
         } // it it reaches to the limitation of retransmission
      } // if failed to send data
      else
      {
        lRetVal = CAM_DRV_SUCCESS;
        break;
      }// // else from if failed to send data

      msleep(3);//delay 3ms before retransmitting again
    } // for loop
  } // if pointers not NULL
  else
  {
    VDB_LOG_ERROR("NULL parameter.\n");
    lRetVal = CAM_DRV_FAILURE;
  } // else from if pointers not NULL

  return lRetVal;
} // LinuxI2cWrite()

//*****************************************************************************

static int32_t LinuxI2cRead(I2cXfrSingle_t *apXfr)
{
  int32_t            lRetVal     = CAM_DRV_SUCCESS;
  struct i2c_client *lpI2cClient = apXfr->mDestDev->mI2cClient;
  uint32_t           lAddrBytes  = 1; // by default single byte reg address
  uint32_t           lDataBytes  = 0;
  uint8_t           *lpData      = NULL;
  uint32_t           lLoop = 0;

  if(apXfr->mByteCnt <= CAM_I2C_SINGLE_DATA_SZ)
  {
    lpData = (uint8_t*)(&apXfr->mData);
  } // if small data
  else
  {
    lpData = (uint8_t*)(apXfr->mData);
  } // else from if small data
  
  if(apXfr->mDestDev->mCfg.mRegAddrWidth == I2C_REG_ADDR_WIDTH_16)
  {
    lAddrBytes = 2;
  } // if reg address has 2 bytes  
  
  // get number of data bytes
  lDataBytes = apXfr->mByteCnt - lAddrBytes;
  
  if(lpI2cClient != NULL)
  {
    int32_t lNumBytes = 0;

    for (lLoop = sI2cRepeat ; lLoop >= 0 ; lLoop--)
    {
      if (lAddrBytes != (int32_t)i2c_master_send(lpI2cClient, 
                                       (char *)lpData, 
                                       lAddrBytes)) 
      {
        if (0 == lLoop)
        {
          lRetVal = CAM_DRV_FAILURE;
          VDB_LOG_ERROR("R err:device_address=0x%x, i2c_bus_idx=%d, address W failed:data=0x%x|%x|%x|%x, ret val:%d\n",
                  lpI2cClient->addr, apXfr->mDestDev->mCfg.mI2cBusIdx, 
                  lpData[0],  lpData[1], lpData[2], lpData[3],
                   lNumBytes);
          break;
        } // it it reaches to the limitation of retransmission
      } // if failed to send read address
      else 
      {
        lpData += lAddrBytes;
        if (lDataBytes != (int32_t)i2c_master_recv(lpI2cClient, 
                                         (char*)lpData, 
                                         lDataBytes))
        {
          if (0 == lLoop)
          {
            lRetVal = CAM_DRV_FAILURE; 
            VDB_LOG_ERROR("R err:device_address=0x%x, i2c_bus_idx=%d, data R failed:data=0x%x|%x|%x|%x, ret val:%d\n",
                    lpI2cClient->addr, apXfr->mDestDev->mCfg.mI2cBusIdx, 
                    lpData[0],  lpData[1], lpData[2], lpData[3],
                    lNumBytes); 
            break;
          } // it it reaches to the limitation of retransmission
          else
          {
            //retransmit i2c transaction
            lpData -= lAddrBytes;// set lpData to the position of the address bytes.
          }// else from it it reaches to the limitation of retransmission
        } // if failed to read data
        else
        {
          lRetVal = CAM_DRV_SUCCESS;
          break;
        } // else from if failed to read data
      } // else from if failed to send read address

      msleep(3);//delay 3ms before retransmitting again
    } // for loop
  } // if pointers not NULL
  else
  {
    VDB_LOG_ERROR("NULL parameter.\n");
    lRetVal = CAM_DRV_FAILURE;
  } // else from if pointers not NULL
  
  return lRetVal;
} //LinuxI2cRead()

//*****************************************************************************

#endif // ifdef __KERNEL__

/*EOF*/