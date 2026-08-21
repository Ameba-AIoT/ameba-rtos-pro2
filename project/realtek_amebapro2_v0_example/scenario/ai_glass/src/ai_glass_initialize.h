#ifndef __AI_GLASS_INITIALIZE_H__
#define __AI_GLASS_INITIALIZE_H__

/******************************************************************************
 *
 * Copyright(c) 2007 - 2015 Realtek Corporation. All rights reserved.
 *
 *
 ******************************************************************************/
void ai_glass_init(void);

extern int ai_glass_disk_reformat(void);
extern volatile uint8_t cancel_wifi_upgrade;
//This is for protection against accidental powerdown from 8773 once critical processes started
extern volatile int critical_process_started;

extern EventGroupHandle_t s_lifetime_event;

extern void ai_glass_init_external_disk(void);
extern void ai_glass_init_ram_disk(void);

#define EXTDISK_LOG   0
#if EXTDISK_LOG
// Functions to manage eMMC logging
void ai_glass_extdisk_log_start(void);
void ai_glass_extdisk_log_stop(void);
#endif

#endif //#ifndef __AI_GLASS_INITIALIZE_H__
