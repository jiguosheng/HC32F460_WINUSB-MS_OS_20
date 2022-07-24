/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file usb_dev_cdc_class.c
 **
 ** A detailed description is available at
 ** @link
      This file provides the CDC VCP core functions.
  @endlink
 ** @brief   This file provides the high layer firmware functions to manage the
 **          following functionalities of the USB CDC Class:
 **           - Initialization and Configuration of high and low layer
 **           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
 **           - OUT/IN data transfer
 **           - Command IN transfer (class requests management)
 **           - Error management
 **
 **   - 2021-04-16  CDT   First version for USB CDC VCP demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_dev_cdc_class.h"
#include "usb_dev_desc.h"
#include "usb_dev_stdreq.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
/*********************************************
   CDC Device library callbacks
 *********************************************/
void usb_dev_cdc_init(void *pdev);
void usb_dev_cdc_deinit(void *pdev);
uint8_t usb_dev_cdc_setup(void *pdev, USB_SETUP_REQ *req);
void usb_dev_cdc_ctrlep_rxready(void *pdev);
void usb_dev_cdc_datain(void *pdev, uint8_t epnum);
void usb_dev_cdc_dataout(void *pdev, uint8_t epnum);
uint8_t *usb_dev_cdc_getcfgdesc(uint16_t *length);

extern __USB_ALIGN_BEGIN uint8_t usb_dev_strdesc[USB_MAX_STR_DESC_SIZ] __USB_ALIGN_END;

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
__USB_ALIGN_BEGIN static uint32_t  alternate_setting  __USB_ALIGN_END = 0ul;
__USB_ALIGN_BEGIN uint8_t usb_rx_buffer[MAX_CDC_PACKET_SIZE] __USB_ALIGN_END ;
uint8_t uart_rx_buffer[APP_RX_DATA_SIZE];  //used as a buffer for receiving data from uart port
__USB_ALIGN_BEGIN uint8_t CmdBuff[CDC_CMD_PACKET_SZE] __USB_ALIGN_END ;
uint32_t APP_Rx_ptr_in  = 0ul;
uint32_t APP_Rx_ptr_out = 0ul;
uint32_t APP_Rx_length  = 0ul;
uint8_t  USB_Tx_State   = 0u;
//static uint32_t cdcCmd  = 0xFFul;
//static uint32_t cdcLen  = 0ul;
static uint32_t LastPackLen = 0;
uint8_t *USBD_WinUSBOSStrDescriptor(void *pdev, uint8_t index,  uint16_t *length);

usb_dev_class_func  class_cdc_cbk =
{
    &usb_dev_cdc_init,
    &usb_dev_cdc_deinit,
    &usb_dev_cdc_setup,
    NULL,
    &usb_dev_cdc_ctrlep_rxready,
    &usb_dev_cdc_getcfgdesc,
    NULL,
    &usb_dev_cdc_datain,
    &usb_dev_cdc_dataout,
    NULL,
    NULL,
		USBD_WinUSBOSStrDescriptor,
};
#define WINUSB_CONFIG_DESC_SIZE 32

__USB_ALIGN_BEGIN uint8_t usb_dev_cdc_cfgdesc[USB_CDC_CONFIG_DESC_SIZ]  __USB_ALIGN_END =
{
		/*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
  WINUSB_CONFIG_DESC_SIZE,                /* wTotalLength:no of returned bytes */
  0x00,
  0x01,   /* bNumInterfaces: 1 interface for WINUSB */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 50*2 mA */
  
  /*---------------------------------------------------------------------------*/
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  0x00,   /* bInterfaceNumber: Number of Interface, zero based index of this interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0xff,   /* bInterfaceClass: vendor */
//  0x00,   /* bInterfaceClass: vendor */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(MAX_CDC_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(MAX_CDC_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_IN_EP,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(MAX_CDC_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(MAX_CDC_PACKET_SIZE),
  0x00                               /* bInterval: ignore for Bulk transfer */
} ;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Initilaize the CDC application
 ** \param  pdev: Device instance
 ** \retval none
 ******************************************************************************/
void usb_dev_cdc_init(void *pdev)
{
    hd_usb_opendevep(pdev, CDC_IN_EP, MAX_CDC_IN_PACKET_SIZE, USB_EP_BULK);
    hd_usb_opendevep(pdev, CDC_OUT_EP, MAX_CDC_OUT_PACKET_SIZE, USB_EP_BULK);
//    hd_usb_opendevep(pdev, CDC_CMD_EP, CDC_CMD_PACKET_SZE, USB_EP_INT);
    
    hd_usb_readytorx(pdev, CDC_OUT_EP, (uint8_t*)(usb_rx_buffer), MAX_CDC_OUT_PACKET_SIZE);
}

/**
 *******************************************************************************
 ** \brief  Deinitialize the CDC application
 ** \param  pdev: Device instance
 ** \retval none
 ******************************************************************************/
void usb_dev_cdc_deinit(void *pdev)
{
    hd_usb_shutdevep(pdev, CDC_IN_EP);
    hd_usb_shutdevep(pdev, CDC_OUT_EP);
//    hd_usb_shutdevep(pdev, CDC_CMD_EP);
}

/**
 *******************************************************************************
 ** \brief  Handle the setup requests
 ** \param  pdev: Device instance
 ** \param  req: usb requests
 ** \retval status
 ******************************************************************************/
uint8_t usb_dev_cdc_setup(void *pdev, USB_SETUP_REQ *req)
{
    uint16_t len=USB_CDC_DESC_SIZ;
    uint8_t  *pbuf=usb_dev_cdc_cfgdesc + 9;
    uint8_t u8Res = USBD_OK;

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
        case USB_REQ_TYPE_CLASS :
            if (req->wLength)
            {
                if (req->bmRequest & 0x80u)
                {
                    
//                    hd_usb_ctrldatatx(pdev, CmdBuff, req->wLength);
                }
                else
                {
//                    cdcCmd = req->bRequest;
//                    cdcLen = req->wLength;
//                    hd_usb_ctrldatarx(pdev, CmdBuff, req->wLength);
                }
            }
            else
            {
               
            }
            break;
        case USB_REQ_TYPE_STANDARD:
            switch (req->bRequest)
            {
                case USB_REQ_GET_DESCRIPTOR:
                    if((req->wValue>>8) == CDC_DESCRIPTOR_TYPE)
                    {
                        pbuf = usb_dev_cdc_cfgdesc + 9u + (9u * USBD_ITF_MAX_NUM);
                        len  = __MIN(USB_CDC_DESC_SIZ , req->wLength);
                    }
                    hd_usb_ctrldatatx(pdev, pbuf, len);
                    break;

                case USB_REQ_GET_INTERFACE :
                    hd_usb_ctrldatatx(pdev, (uint8_t *)&alternate_setting, 1u);
                    break;

                case USB_REQ_SET_INTERFACE :
                    if ((uint8_t)(req->wValue) < USBD_ITF_MAX_NUM)
                    {
                        alternate_setting = (uint8_t)(req->wValue);
                    }
                    else
                    {
                        hd_usb_ctrlerr(pdev);
                    }
                    break;
                default:
                    break;
            }
            break;

        default:
            hd_usb_ctrlerr (pdev);
            u8Res = USBD_FAIL;
            break;
    }
    return u8Res;
}

/**
 *******************************************************************************
 ** \brief  Data received on control endpoint
 ** \param  pdev: device device instance
 ** \retval none
 ******************************************************************************/
void usb_dev_cdc_ctrlep_rxready(void *pdev)
{
    
}

/**
 *******************************************************************************
 ** \brief  Data sent on non-control IN endpoint
 ** \param  pdev: Device instance
 ** \param  epnum: endpoint index
 ** \retval none
 ******************************************************************************/
void usb_dev_cdc_datain(void *pdev, uint8_t epnum)
{
    uint16_t tx2usb_ptr;
    uint16_t tx2usb_length;

    if (USB_Tx_State == 1u)
    {
        if (APP_Rx_length == 0u)
        {
            if(LastPackLen == MAX_CDC_IN_PACKET_SIZE)
            {
                hd_usb_deveptx(pdev, CDC_IN_EP, NULL, 0ul);
                LastPackLen = 0ul;
            }
            else
            {
                USB_Tx_State = 0u;
            }
        }
        else
        {
            if (APP_Rx_length >= MAX_CDC_IN_PACKET_SIZE)
            {
                tx2usb_ptr      = (uint16_t)APP_Rx_ptr_out;
                tx2usb_length   = (uint16_t)MAX_CDC_IN_PACKET_SIZE-1;
                APP_Rx_ptr_out += MAX_CDC_IN_PACKET_SIZE-1;
                APP_Rx_length  -= MAX_CDC_IN_PACKET_SIZE-1;
            }
            else
            {
                tx2usb_ptr      = (uint16_t)APP_Rx_ptr_out;
                tx2usb_length   = (uint16_t)APP_Rx_length;
                APP_Rx_ptr_out += APP_Rx_length;
                APP_Rx_length   = 0u;
            }
            hd_usb_deveptx(pdev,
                           CDC_IN_EP,
                           (uint8_t*)&uart_rx_buffer[tx2usb_ptr],
                           (uint32_t)tx2usb_length);
            LastPackLen = (uint32_t)tx2usb_length;
        }
    }
}
extern unsigned char f;
/**
 *******************************************************************************
 ** \brief  Data received on non-control Out endpoint
 ** \param  pdev: device instance
 ** \param  epnum: endpoint index
 ** \retval none
 ******************************************************************************/
void usb_dev_cdc_dataout(void *pdev, uint8_t epnum)
{
    uint16_t usb_rx_cnt;

    usb_rx_cnt = (uint16_t)((usb_core_instance*)pdev)->dev.out_ep[epnum].xfer_count;
    //vcp_rxdata(usb_rx_buffer, usb_rx_cnt);
		hd_usb_deveptx(pdev,
                          CDC_IN_EP,
                          (uint8_t*)&usb_rx_buffer,
                          usb_rx_cnt);
		f = ~f;
    hd_usb_readytorx(pdev, CDC_OUT_EP, (uint8_t*)(usb_rx_buffer), MAX_CDC_OUT_PACKET_SIZE);
}

/**
 *******************************************************************************
 ** \brief  Start Of Frame event management
 ** \param  pdev: Device instance
 ** \retval status
 ******************************************************************************/
uint8_t usb_dev_cdc_sof(void *pdev)
{
    
    return USBD_OK;
}

/**
 *******************************************************************************
 ** \brief  get the configuration descriptor
 ** \param  length : length of configuration descriptor in bytes
 ** \retval the pointer to configuration descriptor buffer
 ******************************************************************************/
uint8_t *usb_dev_cdc_getcfgdesc(uint16_t *length)
{
    *length = (uint16_t)sizeof (usb_dev_cdc_cfgdesc);
    return usb_dev_cdc_cfgdesc;
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
const uint8_t USBD_OS_STRING[8] = { 
   'M',
   'S',
   'F',
   'T',
   '1',
   '0',
   '0',
   0xA0U, 
}; 

uint8_t *USBD_WinUSBOSStrDescriptor(void *pdev, uint8_t index,  uint16_t *length)
{
  if(index == 0xEE)
	{		// OS String 
          hd_usb_getstring((uint8_t *)USBD_OS_STRING, usb_dev_strdesc, length);
	}
   
   return usb_dev_strdesc;
}


#define USB_LEN_OS_FEATURE_DESC 0x28
#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */

#define USB_BOS_DESCRIPTOR_TYPE (15)

#define MS_OS_20_DESCRIPTOR_LENGTH (0x48)
__USB_ALIGN_BEGIN uint8_t USBD_WINUSB_OSFeatureDesc[MS_OS_20_DESCRIPTOR_LENGTH] __USB_ALIGN_END =
{
   //
 // Microsoft OS 2.0 Descriptor Set Header
 //
 0x0A, 0x00, // wLength - 10 bytes
 0x00, 0x00, // MSOS20_SET_HEADER_DESCRIPTOR
 0x00, 0x00, 0x03, 0x06, // dwWindowsVersion 每 0x06030000 for Windows Blue
 0x48, 0x00, // wTotalLength 每 72 bytes
 //
 // Microsoft OS 2.0 Registry Value Feature Descriptor
 //
 0x3E, 0x00, // wLength - 62 bytes
 0x04, 0x00, // wDescriptorType 每 4 for Registry Property
 0x04, 0x00, // wPropertyDataType - 4 for REG_DWORD
 0x30, 0x00, // wPropertyNameLength 每 48 bytes
 0x53, 0x00, 0x65, 0x00, // Property Name - ※SelectiveSuspendEnabled§
 0x6C, 0x00, 0x65, 0x00,
 0x63, 0x00, 0x74, 0x00,
 0x69, 0x00, 0x76, 0x00,
 0x65, 0x00, 0x53, 0x00,
 0x75, 0x00, 0x73, 0x00,
 0x70, 0x00, 0x65, 0x00,
 0x6E, 0x00, 0x64, 0x00,
 0x45, 0x00, 0x6E, 0x00,
 0x61, 0x00, 0x62, 0x00,
 0x6C, 0x00, 0x65, 0x00,
 0x64, 0x00, 0x00, 0x00,
 0x04, 0x00, // wPropertyDataLength 每 4 bytes
 0x01, 0x00, 0x00, 0x00 // PropertyData - 0x00000001
};
#define USB_LEN_OS_PROPERTY_DESC 0x1C

__USB_ALIGN_BEGIN uint8_t USBD_WINUSB_OSPropertyDesc[USB_LEN_OS_PROPERTY_DESC] __USB_ALIGN_END =
{
   //
 // Microsoft OS 2.0 Platform Capability Descriptor Header
 //
 0x1C, // bLength - 28 bytes
 0x10, // bDescriptorType - 16
 0x05, // bDevCapability 每 5 for Platform Capability
 0x00, // bReserved - 0
 0xDF, 0x60, 0xDD, 0xD8, // MS_OS_20_Platform_Capability_ID -
 0x89, 0x45, 0xC7, 0x4C, // {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F}
 0x9C, 0xD2, 0x65, 0x9D, // 
 0x9E, 0x64, 0x8A, 0x9F, //
 //
 // Descriptor Information Set for Windows 8.1 or later
 //
 0x00, 0x00, 0x03, 0x06, // dwWindowsVersion 每 0x06030000 for Windows Blue
 0x48, 0x00, // wLength 每 size of MS OS 2.0 descriptor set
 0x01, // bMS_VendorCode
 0x00, // bAltEnumCmd 每 0 Does not support alternate enum
};

uint8_t *USBD_WinUSBOSFeatureDescriptor(uint16_t *length)
{
  *length = USB_LEN_OS_FEATURE_DESC;
  return USBD_WINUSB_OSFeatureDesc;
}
uint8_t *USBD_WinUSBOSPropertyDescriptor(uint16_t *length)
{
  *length = USB_LEN_OS_PROPERTY_DESC;
   return USBD_WINUSB_OSPropertyDesc;
}

void USBD_WinUSBGetDescriptor(usb_core_instance *pdev, USB_SETUP_REQ *req)
{
  uint16_t len;
  uint8_t *pbuf;
  
    
  switch (req->wIndex)
  { 
  case 0x04: // compat ID
    USBD_WinUSBOSFeatureDescriptor(&len);
    break;
  case 0x05:
    USBD_WinUSBOSPropertyDescriptor(&len);
    break;
     
  default: 
     hd_usb_ctrlerr(pdev);
    return;
  }
  if((len != 0)&& (req->wLength != 0))
  {
    
    len = MIN(len , req->wLength);
    
    hd_usb_ctrldatatx(pdev, 
                      pbuf,
                      len);
  } 
}   
