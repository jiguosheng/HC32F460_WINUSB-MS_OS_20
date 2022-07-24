/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file main.c
 **
 ** \brief USB composite device(HID+CDC) example.
 **
 **   - 2021-04-14  Linsq First version for USB composite device demo.
 **
 ******************************************************************************/

#include "hc32_ddl.h"
#include "usb_dev_user.h"
#include "usb_dev_desc.h"
#include "usb_bsp.h"
#include "usb_dev_cdc_class.h"

unsigned char ttx[] = "HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32\
HELLOSTM32HELLOSTM32HELLOSTM32HELLOSTM32";
unsigned char f = 0;
unsigned int c = 0;

usb_core_instance  usb_dev;

int32_t main (void)
{
    hd_usb_dev_init(&usb_dev, &user_desc, &class_cdc_cbk, &user_cb);
    while (1)
    {
			if(f != 0)
			{
				hd_usb_deveptx(&usb_dev,CDC_IN_EP,(uint8_t*)&ttx,(uint32_t)sizeof(ttx));
				c = c + sizeof(ttx);
			}
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/