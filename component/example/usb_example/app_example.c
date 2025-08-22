/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "usb_example.h"
#include "stdio.h"
void app_example(void)
{
	/* 	extern void usb_switch_demo(void);
		usb_switch_demo(); */
	printf("Enter atcmd for USB\r\n");
	atcmd_usb_init();
}
