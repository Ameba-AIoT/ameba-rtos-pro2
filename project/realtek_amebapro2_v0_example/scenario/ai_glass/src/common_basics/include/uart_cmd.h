#ifndef __UART_CMD_H__
#define __UART_CMD_H__

/******************************************************************************
 *
 * Copyright(c) 2007 - 2015 Realtek Corporation. All rights reserved.
 *
 *
 ******************************************************************************/
#include "wifi_structures.h"
#include "uart_service.h"

typedef int (*file_seek_func)(FILE *ai_snapshot_rfile, uint32_t file_offset);
typedef int (*file_read_func)(uint8_t *buf, uint32_t read_size, FILE *ai_snapshot_rfile);
typedef int (*file_eof_func)(FILE *ai_snapshot_rfile);

// For UART_RX_OPC_CMD_QUERY_INFO
int uart_resp_get_query_info(uartcmdpacket_t *param);

// For UART_RX_OPC_CMD_POWER_DOWN
int uart_resp_get_power_down(uartcmdpacket_t *param, uint8_t result);

// For UART_RX_OPC_CMD_GET_POWER_STATE
int uart_resp_get_power_state(uartcmdpacket_t *param, uint8_t power_result);

// For UART_RX_OPC_CMD_UPDATE_WIFI_INFO
void *uart_parser_wifi_info_video_info(uartcmdpacket_t *param, uint8_t *info_mode, uint16_t *info_size);
int uart_resp_update_wifi_info(uartcmdpacket_t *param, uint8_t resp_stat);

// For UART_RX_OPC_CMD_SET_GPS
void uart_parser_gps_data(uartcmdpacket_t *param, uint32_t *gps_week, uint32_t *gps_seconds, float *gps_latitude, float *gps_longitude, float *gps_altitude);
int uart_resp_gps_data(uartcmdpacket_t *param, uint8_t status);

// For UART_RX_OPC_CMD_SNAPSHOT
void *uart_parser_snapshot_video_info(uartcmdpacket_t *param, uint8_t *info_mode);
int uart_resp_snapshot(uartcmdpacket_t *param, uint8_t resp_stat);

// For UART_RX_OPC_CMD_GET_FILE_NAME
int uart_resp_get_file_name(uartcmdpacket_t *param, const char *file_name, uint32_t file_length, uint8_t status);

// For UART_RX_OPC_CMD_GET_PICTURE_DATA
int uart_resp_get_pic_data(uartcmdpacket_t *param, FILE *ai_snapshot_rfile, file_seek_func file_seek, file_read_func file_read, file_eof_func file_eof);

// For UART_RX_OPC_CMD_TRANS_PIC_STOP
int uart_resp_get_trans_pic_stop(uartcmdpacket_t *param);

// For UART_RX_OPC_CMD_RECORD_START
uint8_t *uart_parser_recording_video_info(uartcmdpacket_t *param, uint8_t *record_filename_length);
int uart_resp_record_start(uint8_t resp_stat);

// For UART_TX_OPC_RESP_RECORD_CONT
int uart_resp_record_cont(uint8_t resp_stat);

// For UART_RX_OPC_CMD_RECORD_SYNC_TS
int uart_resp_record_sync_ts(uartcmdpacket_t *param);

// For UART_TX_OPC_RESP_RECORD_STOP
int uart_resp_record_stop(uint8_t resp_stat);

// For UART_RX_OPC_CMD_GET_FILE_CNT
int uart_resp_get_file_cnt(uartcmdpacket_t *param, uint16_t film_num, uint16_t snapshot_num, uint8_t result);

// For UART_RX_OPC_CMD_DELETE_FILE
int uart_resp_delete_file(uartcmdpacket_t *param);

// For UART_RX_OPC_CMD_DELETE_ALL_FILES Todo
int uart_resp_delete_all_file(uint8_t resp_stat);

// For UART_RX_OPC_CMD_GET_SD_INFO
int uart_resp_get_sd_info(uartcmdpacket_t *param, uint32_t total_Kbytes, uint32_t used_Kbytes);

// For UART_RX_OPC_CMD_SET_WIFI_MODE
int uart_resp_set_ap_mode(uartcmdpacket_t *param, rtw_softap_info_t *wifi_cfg, uint32_t ssid_buf_size, uint32_t password_buf_size, uint8_t result);

// For UART_RX_OPC_CMD_SET_STA_MODE
int uart_resp_set_sta_mode(uartcmdpacket_t *param, uint8_t status, uint8_t ip_index0, uint8_t ip_index1, uint8_t ip_index2, uint8_t ip_index3);

// For UART_RX_OPC_CMD_GET_PICTURE_DATA_SLIDING_WINDOW
int uart_resp_get_pic_data_sliding_window(uartcmdpacket_t *param, FILE *ai_snapshot_rfile, file_seek_func file_seek, file_read_func file_read,
		file_eof_func file_eof);

// For UART_RX_OPC_CMD_GET_PICTURE_DATA_SLIDING_WINDOW_ACK
int uart_resp_get_pic_data_sliding_window_ack(uartcmdpacket_t *param);

// For UART_RX_OPC_CMD_SET_SYS_UPGRADE
typedef struct {
	uint8_t upgradetype;
	uint8_t version[4];
} UpgradeInfo;
UpgradeInfo uart_parser_version_and_upgradetype(uartcmdpacket_t *param);
void *uart_parser_version(uartcmdpacket_t *param, uint8_t *version);
int uart_resp_sys_upgrade(uint8_t resp_stat);
int uart_resp_request_sys_upgrade(uint8_t resp_stat);

// For UART_TX_OPC_CMD_TRANSFER_UPGRADE_DATA
int uart_resp_transfer_upgrade_data(uint8_t *data_buffer, uint16_t data_length);
int uart_resp_finish_bt_soc_fw_upgrade(void);

// For UART_RX_OPC_CMD_SET_WIFI_FW_ROLLBACK
int uart_resp_set_wifi_fw_rollback(uint8_t resp_stat);

// For UART_RX_OPC_RESP_START_BT_SOC_FW_UPGRADE
int uart_resp_start_bt_soc_fw_upgrade_ack(uint8_t resp_stat);

// For UART_RX_OPC_CMD_GET_SYS_UPGRADE
int uart_resp_get_sys_upgrade(uint8_t device_id, uint8_t device_upgrade_status);

// UART_RX_OPC_CMD_CANCEL_SYS_UPGRADE
int uart_resp_cancel_sys_upgrade(uint8_t resp_stat);

#endif //#ifndef __UART_CMD_H__
