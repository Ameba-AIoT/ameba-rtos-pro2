#ifndef APP_SETTING_H
#define APP_SETTING_H

#include "basic_types.h"

/*doorbell-chime mode*/
#define DOORBELL_CHIME_MODE_ENABLE   1
//#define USE_EVB                    1
//#define QFN_128_BOARD                1

/* --------- LS Setting --------- */
#if DOORBELL_CHIME_MODE_ENABLE
#define DOORBELL_CHIME_BROAD        1
#define USE_UART_COMMUNICATION      1
#define USE_GPIO_WATCH_DOG          1
#define USE_GPIO_QUICK_EVENT        1

#define GPIO_WDT_PIN             PG_9  //output
#define GPIO_EVENT_INT_PIN       PG_8  //irq
#define GPIO_EVENT0_PIN          PG_0  //input
#define GPIO_EVENT1_PIN          PG_1  //input
#define GPIO_EVENT2_PIN          PG_2  //input
#define UART_TX                  PC_9
#define UART_RX                  PC_8
#define LED2                     PG_3


typedef struct connect_ctr_s {
	uint32_t wifi_state;
	uint32_t bt_state;
} connect_ctr_t;

#if USE_EVB
#define USE_ICC_CMD        1
#define USE_PUSH_BUTTON    1
#define POWER_SAVE_MODE    1

/* --------- ICC command --------*/
#define ICC_CMD_SHORT    0x10 //RING
#define ICC_CMD_POWEROFF 0x13
#define DOORBELL_AMP_ENABLE	0
#else
#define DOORBELL_AMP_ENABLE	1
#endif


#else

#define USE_ICC_CMD        1
#define USE_PIR_SENSOR     1
#define USE_BATTERY_DETECT 0
#define USE_PUSH_BUTTON    1
#define USE_WOWLAN         1

#define DOORBELL_AMP_ENABLE	0

/* --------- Power save mode ---- */
#define POWER_SAVE_MODE    1 //0:deepsleep, 1:standby, 2:sleepPG

/* --------- ICC command --------*/
#define ICC_CMD_SHORT    0x10 //RING
#define ICC_CMD_LONG     0x11 //QRCODE 
#define ICC_CMD_POWERON  0x12
#define ICC_CMD_POWEROFF 0x13
#define ICC_CMD_RTC      0x14
#define ICC_CMD_RTC_SET  0x15
#define ICC_CMD_PIR      0x16
#define ICC_CMD_BATTOFF  0x17

#define ICC_CMD_REQ_GET_LS_WAKE_REASON          (0x30)
#define ICC_CMD_NOTIFY_LS_WAKE_REASON           (0x31)

#endif //DOORBELL_CHIME_MODE_ENABLE

/* -------- doorbell state ------*/
#define STATE_NORMAL          0x00
#define STATE_WIFI_CONNECTED  0x01
#define STATE_SD_INSERT       0x02
#define STATE_MEDIA_READY     0x04
#define STATE_RESERVE2        0x08

#define STATE_QRCODE          0x10
#define STATE_RING            0x20
#define STATE_ARAM            0x40
#define STATE_RESERVE3        0x80

#define STATE_RECORD          0x100
#define STATE_PLAYBACK        0x200
#define STATE_NONESD          0x400
#define STATE_RESERVE5        0x800

#define STATE_STREAM          0x1000
#define STATE_SPEAK           0x2000

#define RESET_TRIGGER         0x01
#define CALL_TRIGGER          0x02
#define PIR_TRIGGER           0x04
#define RF_TRIGGER            0x08

//wakeup source setting
typedef struct wakeup_source_s {
	u16 Option;
	u32 SDuration;
	int32_t gpio_reg;
} wakeup_source_t;

typedef struct doorbell_ctr_s {
	//doorbell current state
	uint32_t doorbell_state;
	uint32_t new_state;

	//STREAM and speak times
	uint8_t speak_on;
	uint8_t stream_on;

} doorbell_ctr_t;

/* -------- PIN define ---------*/
#if USE_PIR_SENSOR
#ifdef QFN_128_BOARD
#define GPIO_SERIN PA_2
#define GPIO_DLA PA_7
#else
#define GPIO_SERIN       PA_2
#define GPIO_DLA         PA_3
#endif
#endif

#if USE_PUSH_BUTTON
#define BTN_PIN          PA_13
#endif

#if USE_BATTERY_DETECT
#define ADC_PIN          PA_4
#define ADC_THRESHOLD    0x308
#endif

#define AMP_PIN PE_0

#define BTN_RED  PC_9
#define BTN_BLUE PC_8

/* ------- application define ----*/

#define SNAPSHOT_DIR     "PHOTO"
#define MP4_DIR          "VIDEO"

#endif //APP_SETTING_H