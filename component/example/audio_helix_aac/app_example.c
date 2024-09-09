/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "example_audio_helix_aac.h"

void app_example(void)
{
#ifdef __ICCARM__
	extern void example_audio_aac();
	example_audio_aac();
#else
	example_audio_helix_aac();
#endif
}
