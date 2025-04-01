#ifndef __UART_SERVICE_H__
#define __UART_SERVICE_H__

/******************************************************************************
 *
 * Copyright(c) 2007 - 2015 Realtek Corporation. All rights reserved.
 *
 *
 ******************************************************************************/
#define UART_PROTOCAL_VER       0		// The uart protocal version
#define UART_WIFI_IC_TYPE       0		// The uart wifi ic type

#define EMPTY_PACKET_LEN        5                           // sync_word (1), seq_number(1), length(2), checksum(1)
#define EMPTY_PACKET_ACK_LEN    9                           // sync_word (1), seq_number(1), length(2), checksum(1), get_ts(4)
#define ACK_DATA_LEN            5                           // opcode(2), status(1), get_opcode(2)
#define UART_MIN_BUF_SIZE       (256 + EMPTY_PACKET_LEN)    // The min size for buffer check
#define UART_MAX_BUF_SIZE       (1536 + EMPTY_PACKET_LEN)   // The max size for buffer check
#define UART_MIN_PIC_SIZE       (256 + EMPTY_PACKET_LEN)    // The min size for one picture packet
#define UART_MAX_PIC_SIZE       (1536 + EMPTY_PACKET_LEN)   // The max size for one picture packet
#define UART_DEFAULT_BUF_SIZE   (1536 + EMPTY_PACKET_LEN)   // The default size for buffer check
#define UART_DEFAULT_PIC_SIZE   (1536 + EMPTY_PACKET_LEN)   // The default size for one picture packet

#define IS_VALID_UART_BUF_SIZE(value) \
    ((value) >= UART_MIN_BUF_SIZE || \
     (value) <= UART_MAX_BUF_SIZE)

#define IS_VALID_UART_PIC_SIZE(value) \
    ((value) >= UART_MIN_PIC_SIZE || \
     (value) <= UART_MAX_PIC_SIZE)

#define SEND_DATA_SHOW      0 // Show the data when the data sendout
#define RECEIVE_ACK_SHOW    0 // Show the receive ack
#define SEND_ACK_SHOW       0 // Show the send ack

#define MAX_UART_QUEUE_SIZE		10
#define MAX_CRITICAL_QUEUE_SIZE	10
#define MAX_UARTACK_QUEUE_SIZE	20
// Uart Opcode CMD
typedef enum {
	UART_RX_OPC_ACK                     = 0x0000,
	UART_RX_OPC_MIN                             = 0x0631, // define the min value of opcode command
	UART_RX_OPC_RESP_START_BT_SOC_FW_UPGRADE    = 0x0631,
	//UART_RX_OPC_RESP_TRANSFER_UPGRADE_DATA      = 0x0632,
	UART_RX_OPC_RESP_FINISH_BT_SOC_FW_UPGRADE   = 0x0633,
	UART_RX_OPC_CMD_QUERY_INFO          = 0x8400,
	//UART_RX_OPC_CMD_POWER_ON           = 0x8401,
	UART_RX_OPC_CMD_POWER_DOWN          = 0x8402,
	UART_RX_OPC_CMD_GET_POWER_STATE     = 0x8403,
	UART_RX_OPC_CMD_UPDATE_WIFI_INFO    = 0x8404,
	UART_RX_OPC_CMD_SET_GPS             = 0x8405,
	UART_RX_OPC_CMD_SNAPSHOT            = 0x8406,
	UART_RX_OPC_CMD_GET_FILE_NAME       = 0x8407,
	UART_RX_OPC_CMD_GET_PICTURE_DATA    = 0x8408,
	UART_RX_OPC_CMD_TRANS_PIC_STOP      = 0x8409,
	UART_RX_OPC_CMD_RECORD_START        = 0x840A,
	//UART_RX_OPC_CMD_RECORD_CONT         = 0x840B,
	UART_RX_OPC_CMD_RECORD_SYNC_TS      = 0x840C,
	UART_RX_OPC_CMD_RECORD_STOP         = 0x840D,
	UART_RX_OPC_CMD_GET_FILE_CNT        = 0x840E,
	UART_RX_OPC_CMD_DELETE_FILE         = 0x840F,
	UART_RX_OPC_CMD_DELETE_ALL_FILES    = 0x8410,
	UART_RX_OPC_CMD_GET_SD_INFO         = 0x8411,
	UART_RX_OPC_CMD_SET_WIFI_MODE       = 0x8412,
	UART_RX_OPC_CMD_GET_PICTURE_DATA_SLIDING_WINDOW     = 0x8413,
	UART_RX_OPC_CMD_GET_PICTURE_DATA_SLIDING_WINDOW_ACK = 0x8414,
	UART_RX_OPC_CMD_SET_WIFI_FW_ROLLBACK                = 0x8415,
	UART_RX_OPC_CMD_SET_SYS_UPGRADE     = 0x8430,
	UART_RX_OPC_MAX                     = 0x8430, // define the max value of opcode command
} uart_rx_opc_e;

#define UART_RX_OPC_COUNT (UART_RX_OPC_MAX - UART_RX_OPC_MIN + 1)

#define IS_VALID_UART_RX_OPC(value) \
    ((value) == UART_RX_OPC_ACK || \
     (value) == UART_RX_OPC_RESP_START_BT_SOC_FW_UPGRADE || \
     (value) == UART_RX_OPC_RESP_FINISH_BT_SOC_FW_UPGRADE || \
     (value) == UART_RX_OPC_CMD_QUERY_INFO || \
     (value) == UART_RX_OPC_CMD_POWER_DOWN || \
     (value) == UART_RX_OPC_CMD_GET_POWER_STATE || \
     (value) == UART_RX_OPC_CMD_UPDATE_WIFI_INFO || \
     (value) == UART_RX_OPC_CMD_SET_GPS || \
     (value) == UART_RX_OPC_CMD_SNAPSHOT || \
     (value) == UART_RX_OPC_CMD_GET_FILE_NAME || \
     (value) == UART_RX_OPC_CMD_GET_PICTURE_DATA || \
     (value) == UART_RX_OPC_CMD_TRANS_PIC_STOP || \
     (value) == UART_RX_OPC_CMD_RECORD_START || \
     (value) == UART_RX_OPC_CMD_RECORD_SYNC_TS || \
     (value) == UART_RX_OPC_CMD_RECORD_STOP || \
     (value) == UART_RX_OPC_CMD_GET_FILE_CNT || \
     (value) == UART_RX_OPC_CMD_DELETE_FILE || \
     (value) == UART_RX_OPC_CMD_DELETE_ALL_FILES || \
     (value) == UART_RX_OPC_CMD_SET_WIFI_MODE || \
     (value) == UART_RX_OPC_CMD_GET_SD_INFO || \
     (value) == UART_RX_OPC_CMD_GET_PICTURE_DATA_SLIDING_WINDOW || \
     (value) == UART_RX_OPC_CMD_GET_PICTURE_DATA_SLIDING_WINDOW_ACK || \
     (value) == UART_RX_OPC_CMD_SET_WIFI_FW_ROLLBACK || \
     (value) == UART_RX_OPC_CMD_SET_SYS_UPGRADE)

#define IS_NO_ACK_UART_RX_OPC(value) \
    ((value) == UART_RX_OPC_CMD_GET_PICTURE_DATA_SLIDING_WINDOW_ACK)

#if 0
#define IS_CRITICAL_UART_RX_OPC(value) \
    ((value) == UART_RX_OPC_CMD_POWER_DOWN || \
     (value) == UART_RX_OPC_CMD_GET_POWER_STATE || \
     (value) == UART_RX_OPC_CMD_TRANS_PIC_STOP || \
     (value) == UART_RX_OPC_CMD_RECORD_SYNC_TS || \
     (value) == UART_RX_OPC_CMD_RECORD_STOP)
#else
#define IS_CRITICAL_UART_RX_OPC(value) \
    ((value) == UART_RX_OPC_CMD_QUERY_INFO || \
     (value) == UART_RX_OPC_CMD_POWER_DOWN || \
     (value) == UART_RX_OPC_CMD_GET_POWER_STATE || \
     (value) == UART_RX_OPC_CMD_TRANS_PIC_STOP || \
     (value) == UART_RX_OPC_CMD_RECORD_SYNC_TS || \
     (value) == UART_RX_OPC_CMD_RECORD_STOP)
#endif

// Uart Opcode Response
typedef enum {
	UART_TX_OPC_ACK                     = 0x0000,
	UART_TX_OPC_MIN                     = 0x0631, // define the min value of opcode response
	UART_TX_OPC_CMD_START_BT_SOC_FW_UPGRADE     = 0x0631,
	UART_TX_OPC_CMD_TRANSFER_UPGRADE_DATA       = 0x0632,
	UART_TX_OPC_CMD_FINISH_BT_SOC_FW_UPGRADE    = 0x0633,
	UART_TX_OPC_RESP_QUERY_INFO         = 0x8400,
	UART_TX_OPC_RESP_POWER_ON           = 0x8401,
	//UART_TX_OPC_RESP_POWER_DOWN           = 0x8402,
	UART_TX_OPC_RESP_GET_POWER_STATE    = 0x8403,
	UART_TX_OPC_RESP_UPDATE_WIFI_INFO   = 0x8404,
	UART_TX_OPC_RESP_SET_GPS            = 0x8405,
	UART_TX_OPC_RESP_SNAPSHOT           = 0x8406,
	UART_TX_OPC_RESP_GET_FILE_NAME      = 0x8407,
	UART_TX_OPC_RESP_GET_PICTURE_DATA   = 0x8408,
	UART_TX_OPC_RESP_TRANS_PIC_STOP     = 0x8409,
	UART_TX_OPC_RESP_RECORD_START       = 0x840A,
	UART_TX_OPC_RESP_RECORD_CONT        = 0x840B,
	UART_TX_OPC_RESP_RECORD_SYNC_TS     = 0x840C,
	UART_TX_OPC_RESP_RECORD_STOP        = 0x840D,
	UART_TX_OPC_RESP_GET_FILE_CNT       = 0x840E,
	UART_TX_OPC_RESP_DELETE_FILE        = 0x840F,
	UART_TX_OPC_RESP_DELETE_ALL_FILES   = 0x8410,
	UART_TX_OPC_RESP_GET_SD_INFO        = 0x8411,
	UART_TX_OPC_RESP_SET_WIFI_MODE      = 0x8412,
	UART_TX_OPC_RESP_GET_PICTURE_DATA_SLIDING_WINDOW    = 0x8413,
	UART_TX_OPC_RESP_SET_WIFI_FW_UPGRADE                = 0x8415,
	UART_TX_OPC_RESP_REQUEST_SET_SYS_UPGRADE            = 0x8430,
	UART_TX_OPC_MAX                     = 0x8430, // define the max value of opcode response
} uart_tx_opc_e;

#define UART_TX_OPC_COUNT (UART_TX_OPC_MAX - UART_TX_OPC_MIN + 1)

#define IS_VALID_UART_TX_OPC(value) \
    ((value) == UART_TX_OPC_ACK || \
     (value) == UART_TX_OPC_CMD_START_BT_SOC_FW_UPGRADE || \
     (value) == UART_TX_OPC_CMD_TRANSFER_UPGRADE_DATA || \
     (value) == UART_TX_OPC_CMD_FINISH_BT_SOC_FW_UPGRADE || \
     (value) == UART_TX_OPC_RESP_QUERY_INFO || \
     (value) == UART_TX_OPC_RESP_POWER_ON || \
     (value) == UART_TX_OPC_RESP_GET_POWER_STATE || \
     (value) == UART_TX_OPC_RESP_UPDATE_WIFI_INFO || \
     (value) == UART_TX_OPC_RESP_SET_GPS || \
     (value) == UART_TX_OPC_RESP_SNAPSHOT || \
     (value) == UART_TX_OPC_RESP_GET_FILE_NAME || \
     (value) == UART_TX_OPC_RESP_GET_PICTURE_DATA || \
     (value) == UART_TX_OPC_RESP_TRANS_PIC_STOP || \
     (value) == UART_TX_OPC_RESP_RECORD_START || \
     (value) == UART_TX_OPC_RESP_RECORD_CONT || \
     (value) == UART_TX_OPC_RESP_RECORD_SYNC_TS || \
     (value) == UART_TX_OPC_RESP_RECORD_STOP || \
     (value) == UART_TX_OPC_RESP_GET_FILE_CNT || \
     (value) == UART_TX_OPC_RESP_DELETE_FILE || \
     (value) == UART_TX_OPC_RESP_DELETE_ALL_FILES || \
     (value) == UART_TX_OPC_RESP_SET_WIFI_MODE || \
     (value) == UART_TX_OPC_RESP_GET_SD_INFO || \
     (value) == UART_TX_OPC_RESP_GET_PICTURE_DATA_SLIDING_WINDOW || \
     (value) == UART_TX_OPC_RESP_SET_WIFI_FW_UPGRADE || \
     (value) == UART_TX_OPC_RESP_REQUEST_SET_SYS_UPGRADE)

// ai glass status with bt
typedef enum {
	AI_GLASS_CMD_COMPLETE   = 0x00,
	AI_GLASS_CMD_DISALLOW   = 0x01,
	AI_GLASS_CMD_UNKNOWN    = 0x02,
	AI_GLASS_PARAMS_ERR     = 0x03,
	AI_GLASS_BUSY           = 0x04,
	AI_GLASS_PROC_FAIL      = 0x05,
	AI_GLASS_ONE_WIRE_EXT   = 0x06,
	AI_GLASS_TTS_REQ        = 0x07,
	AI_GLASS_MUSIC_REQ      = 0x08,
	AI_GLASS_VERS_INCOMP    = 0x09,
	AI_GLASS_SCEN_ERR       = 0x0A,
	AI_GLASS_GATT           = 0x11,
	AI_GLASS_GATT_ERR       = 0x12,
	AI_GLASS_SD_FULL        = 0x20,
	AI_GLASS_OTA_FILE_NOT_EXISTED   = 0x30,
	AI_GLASS_OTA_PROCESS_FAILED     = 0x31,
	AI_GALSS_OTA_BATTERY_LOW        = 0x32,
} ai_glass_status_e;

// Uart error code
enum {
	UART_START_FAIL             = -8,
	UART_ACK_PACKET_UNAVAILABLE = -7,
	UART_SEND_PACKET_FAIL       = -6,
	UART_ACK_TIMEOUT            = -5,
	UART_SEND_PACKET_TIMEOUT    = -4,
	UART_REG_CALLBACK_FAIL      = -3,
	UART_QUEUE_CREATE_FAIL      = -2,
	UART_SERVICE_INIT_FAIL      = -1,
	UART_OK                     = 0,
};

// Uart power status
enum {
	UART_PWR_NORMAL             = 0x01,
	UART_PWR_DLPS               = 0x02,
	UART_PWR_OFF                = 0x03,
	UART_PWR_APON               = 0x04,
	UART_PWR_HTTP_CONN          = 0x05,
};

// The uart command packet
typedef struct uartpacket_s {
	uint8_t     sync_word;      // sync word
	uint8_t     seq_number;     // sequence number
	uint16_t    length;         // length for data (packet length - 2)
	uint16_t    opcode;         // opcode for command or event
	uint8_t    *data_buf;       // data buffer
	uint8_t     checksum;       // checksum
} uartpacket_t;

typedef struct uartcmdpacket_s {
	uartpacket_t    uart_pkt;
	uint8_t         exp_seq_number;	// expect_sequence number (keep the inforamtion for future)
} uartcmdpacket_t;

typedef struct uartackpacket_s {
	uint8_t     sync_word;      // sync word
	uint8_t     seq_number;     // sequence number
	uint16_t    length;         // length for data (packet length - 5)
	uint16_t    opcode;         // opcode for command or event
	uint16_t    recv_opcode;
	uint8_t     status;
	uint8_t     checksum;       // checksum
	uint32_t    get_ts;         // record the time get ack packet
} uartackpacket_t;

typedef struct uart_params {
	uint8_t *data;
	uint16_t length;
	struct uart_params *next;
} uart_params_t;

typedef void (*callback_t)(uartcmdpacket_t *param);

typedef struct {
	uint16_t		opcode;
	callback_t		callback;
} callback_entry_t;

extern uint8_t uart_protocal_version;
extern uint16_t uart_buff_size;
extern uint16_t uart_pic_size;
extern uint8_t uart_wifi_ic_type;

int uart_service_rx_cmd_reg(uint16_t uart_information, callback_t uart_cmd_fun);
int uart_service_init(PinName tx, PinName rx, int baudrate);
int uart_service_start(int send_power_start);
int uart_send_packet(uint16_t resp_opcode, uart_params_t *params_head, bool ignore_ack, int timeout);
void uart_service_poweroff(void);
void uart_service_deinit(void);

#endif //#ifndef __UART_SERVICE_H__
