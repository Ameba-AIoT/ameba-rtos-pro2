/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      central_client_app.c
   * @brief     This file handles BLE central application routines.
   * @author    jane
   * @date      2017-06-06
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <platform_opts_bt.h>
#if defined(CONFIG_BT_SCATTERNET) && CONFIG_BT_SCATTERNET
#include "platform_stdlib.h"
#include <app_msg.h>
#include <string.h>
#include <trace_app.h>
#include <gap_scan.h>
#include <gap.h>
#include <gap_adv.h>
#include <gap_msg.h>
#include <gap_bond_le.h>
#include <ble_scatternet_app.h>
#include <ble_scatternet_link_mgr.h>
#include <gcs_client.h>
#include "gatt_builtin_services.h"
#include "ble_central_at_cmd.h"
#include "ble_peripheral_at_cmd.h"
#if defined(APP_LE_EXT_ADV_SCAN_SUPPORT) && APP_LE_EXT_ADV_SCAN_SUPPORT
#include <gap_ext_adv.h>
#include <gap_ext_scan.h>
#include <peripheral_app.h>
#endif
/** @defgroup  CENTRAL_CLIENT_APP Central Client Application
    * @brief This file handles BLE central client application routines.
    * @{
    */
/*============================================================================*
 *                              Variables
 *============================================================================*/
/** @addtogroup  CENTRAL_CLIIENT_CALLBACK
    * @{
    */
T_CLIENT_ID   ble_scatternet_gcs_client_id;         /**< General Common Services client client id*/
/** @} */ /* End of group CENTRAL_CLIIENT_CALLBACK */

/** @defgroup  CENTRAL_CLIENT_GAP_MSG GAP Message Handler
    * @brief Handle GAP Message
    * @{
    */
T_GAP_DEV_STATE ble_scatternet_gap_dev_state = {0, 0, 0, 0, 0};                /**< GAP device state */
int ble_scatternet_peripheral_app_max_links = 0;
int ble_scatternet_central_app_max_links = 0;
#if APP_PRIVACY_EN
T_PRIVACY_STATE ble_scatternet_app_privacy_state = PRIVACY_STATE_INIT;
T_PRIVACY_ADDR_RESOLUTION_STATE ble_scatternet_app_privacy_resolution_state = PRIVACY_ADDR_RESOLUTION_DISABLED;
T_BLE_SCATTERNET_APP_WORK_MODE ble_scatternet_app_work_mode = BLE_SCATTERNET_APP_PAIRABLE_MODE;
#endif
#if defined(APP_LE_EXT_ADV_SCAN_SUPPORT) && APP_LE_EXT_ADV_SCAN_SUPPORT
extern uint8_t random_update_flag;
#endif
/*============================================================================*
 *                              Functions
 *============================================================================*/
void ble_scatternet_app_handle_gap_msg(T_IO_MSG  *p_gap_msg);
/**
 * @brief    All the application messages are pre-handled in this function
 * @note     All the IO MSGs are sent to this function, then the event handling
 *           function shall be called according to the MSG type.
 * @param[in] io_msg  IO message data
 * @return   void
 */
void ble_scatternet_app_handle_io_msg(T_IO_MSG io_msg)
{
	uint16_t msg_type = io_msg.type;

	switch (msg_type) {
	case IO_MSG_TYPE_BT_STATUS: {
		ble_scatternet_app_handle_gap_msg(&io_msg);
	}
	break;
	case IO_MSG_TYPE_AT_CMD: {
		uint16_t subtype = io_msg.subtype;
		void *arg = io_msg.u.buf;
		if (ble_central_app_handle_at_cmd(subtype, arg) != 1) {
			ble_peripheral_app_handle_at_cmd(subtype, arg);
		}
	}
	break;
	case IO_MSG_TYPE_QDECODE: {
		if (io_msg.subtype == 0) {
			le_adv_stop();
		} else if (io_msg.subtype == 1) {
			le_adv_start();
		} else if (io_msg.subtype == 2) {
			le_scan_stop();
		} else if (io_msg.subtype == 3) {
			le_scan_start();
		}
	}
	break;
	default:
		break;
	}
}

#if APP_PRIVACY_EN
/**
  * @brief Application changes to pairable mode.
  * @retval void
  */
void ble_scatternet_app_change_to_pair_mode(void){
	APP_PRINT_INFO0("ble_scatternet_app_change_to_pair_mode");
	printf("ble_scatternet_app change to pair mode\r\n");
	if (ble_scatternet_app_work_mode == BLE_SCATTERNET_APP_RECONNECTION_MODE){
	    if (ble_scatternet_gap_dev_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING){
	        le_adv_stop();
	    }
	}
	ble_scatternet_app_work_mode = BLE_SCATTERNET_APP_PAIRABLE_MODE;
}

/**
 * @brief    Start advertising
 *
 * @return   void
 */
void ble_scatternet_app_adv_start(void)
{
    uint8_t  adv_evt_type = GAP_ADTYPE_ADV_IND;
    uint8_t  adv_filter_policy = GAP_ADV_FILTER_ANY;
    T_LE_KEY_ENTRY *p_entry;
    p_entry = le_get_high_priority_bond();

    if (p_entry == NULL)
    {
        /* No bonded device, send connectable undirected advertisement without using whitelist*/
        ble_scatternet_app_work_mode = BLE_SCATTERNET_APP_PAIRABLE_MODE;
        adv_filter_policy = GAP_ADV_FILTER_ANY;
        if (ble_scatternet_app_privacy_resolution_state == PRIVACY_ADDR_RESOLUTION_ENABLED)
        {
            privacy_set_addr_resolution(false);
        }
    }
    else
    {
        ble_scatternet_app_work_mode = BLE_SCATTERNET_APP_RECONNECTION_MODE;
        adv_filter_policy = GAP_ADV_FILTER_WHITE_LIST_ALL;
        if (ble_scatternet_app_privacy_resolution_state == PRIVACY_ADDR_RESOLUTION_DISABLED)
        {
            privacy_set_addr_resolution(true);
        }
    }
    le_adv_set_param(GAP_PARAM_ADV_EVENT_TYPE, sizeof(adv_evt_type), &adv_evt_type);
    le_adv_set_param(GAP_PARAM_ADV_FILTER_POLICY, sizeof(adv_filter_policy), &adv_filter_policy);
    le_adv_start();
}

/**
  * @brief Callback for BLE privacy management module to notify app
  * @param[in] type     callback msy type @ref T_PRIVACY_CB_TYPE.
  * @param[in] cb_data  callback data.
  * @retval void
  */
void ble_scatternet_app_privacy_callback(T_PRIVACY_CB_TYPE type, T_PRIVACY_CB_DATA cb_data)
{
    APP_PRINT_INFO1("ble_scatternet_app_privacy_callback: type %d", type);
    switch (type)
    {
    case PRIVACY_STATE_MSGTYPE:
        ble_scatternet_app_privacy_state = cb_data.privacy_state;
        APP_PRINT_INFO1("PRIVACY_STATE_MSGTYPE: status %d", ble_scatternet_app_privacy_state);
        break;

    case PRIVACY_RESOLUTION_STATUS_MSGTYPE:
        ble_scatternet_app_privacy_resolution_state = cb_data.resolution_state;
        APP_PRINT_INFO1("PRIVACY_RESOLUTION_STATUS_MSGTYPE: status %d", ble_scatternet_app_privacy_resolution_state);
        break;

    default:
        break;
    }
}
#endif

/**
 * @brief    Handle msg GAP_MSG_LE_DEV_STATE_CHANGE
 * @note     All the gap device state events are pre-handled in this function.
 *           Then the event handling function shall be called according to the new_state
 * @param[in] new_state  New gap device state
 * @param[in] cause GAP device state change cause
 * @return   void
 */
void ble_scatternet_app_handle_dev_state_evt(T_GAP_DEV_STATE new_state, uint16_t cause)
{
	APP_PRINT_INFO3("ble_scatternet_dev_state_evt: init state  %d, scan state %d, cause 0x%x",
					new_state.gap_init_state,
					new_state.gap_scan_state, cause);
#if APP_PRIVACY_EN
	if ((new_state.gap_init_state == GAP_INIT_STATE_STACK_READY)
		&& (new_state.gap_adv_state == GAP_ADV_STATE_IDLE)
		&& (new_state.gap_conn_state == GAP_CONN_DEV_STATE_IDLE))
	{
		privacy_handle_resolv_list();
	}
#endif

	if (ble_scatternet_gap_dev_state.gap_init_state != new_state.gap_init_state) {
		if (new_state.gap_init_state == GAP_INIT_STATE_STACK_READY) {
			uint8_t bt_addr[6];
			APP_PRINT_INFO0("GAP stack ready");
			/*stack ready*/
			gap_get_param(GAP_PARAM_BD_ADDR, bt_addr);
			printf("local bd addr: 0x%2x:%2x:%2x:%2x:%2x:%2x\r\n",
				   bt_addr[5],
				   bt_addr[4],
				   bt_addr[3],
				   bt_addr[2],
				   bt_addr[1],
				   bt_addr[0]);
#if defined(APP_LE_EXT_ADV_SCAN_SUPPORT) && APP_LE_EXT_ADV_SCAN_SUPPORT
			ext_adv_start_t adv_start_param;
			adv_start_param.adv_handle = 0;
			adv_start_param.duration = 0;
			adv_start_param.num_events = 0;
			ble_peripheral_start_ext_adv(&adv_start_param);
#else
#if APP_PRIVACY_EN
			ble_scatternet_app_adv_start();
#else
			le_adv_start();
#endif
#endif
		}
	}

	if (ble_scatternet_gap_dev_state.gap_scan_state != new_state.gap_scan_state) {
		if (new_state.gap_scan_state == GAP_SCAN_STATE_IDLE) {
			APP_PRINT_INFO0("GAP scan stop");
			printf("GAP scan stop\r\n");
		} else if (new_state.gap_scan_state == GAP_SCAN_STATE_SCANNING) {
			APP_PRINT_INFO0("GAP scan start");
			printf("GAP scan start\r\n");
		}
	}

	if (ble_scatternet_gap_dev_state.gap_adv_state != new_state.gap_adv_state) {
		if (new_state.gap_adv_state == GAP_ADV_STATE_IDLE) {
			if (new_state.gap_adv_sub_state == GAP_ADV_TO_IDLE_CAUSE_CONN) {
				APP_PRINT_INFO0("GAP adv stoped: because connection created");
				printf("GAP adv stoped: because connection created\r\n");
			} else {
				APP_PRINT_INFO0("GAP adv stoped");
				printf("GAP adv stopped\r\n");
			}
		} else if (new_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING) {
			APP_PRINT_INFO0("GAP adv start");
			printf("GAP adv start\r\n");
		}
	}

	ble_scatternet_gap_dev_state = new_state;
}

/**
 * @brief    Handle msg GAP_MSG_LE_CONN_STATE_CHANGE
 * @note     All the gap conn state events are pre-handled in this function.
 *           Then the event handling function shall be called according to the new_state
 * @param[in] conn_id Connection ID
 * @param[in] new_state  New gap connection state
 * @param[in] disc_cause Use this cause when new_state is GAP_CONN_STATE_DISCONNECTED
 * @return   void
 */
void ble_scatternet_app_handle_conn_state_evt(uint8_t conn_id, T_GAP_CONN_STATE new_state, uint16_t disc_cause)
{
	T_GAP_CONN_INFO conn_info;

	if (conn_id >= BLE_SCATTERNET_APP_MAX_LINKS) {
		return;
	}

	APP_PRINT_INFO4("ble_scatternet_app_handle_conn_state_evt: conn_id %d, conn_state(%d -> %d), disc_cause 0x%x",
					conn_id, ble_scatternet_app_link_table[conn_id].conn_state, new_state, disc_cause);

	ble_scatternet_app_link_table[conn_id].conn_state = new_state;
	switch (new_state) {
	case GAP_CONN_STATE_DISCONNECTED: {
		if ((disc_cause != (HCI_ERR | HCI_ERR_REMOTE_USER_TERMINATE))
			&& (disc_cause != (HCI_ERR | HCI_ERR_LOCAL_HOST_TERMINATE))) {
			APP_PRINT_ERROR2("ble_scatternet_app_handle_conn_state_evt: connection lost, conn_id %d, cause 0x%x", conn_id,
							 disc_cause);
		}

		printf("Disconnect conn_id %d, cause 0x%x\r\n", conn_id, disc_cause);
///judge the type of disconnect is central or peripheral,if peripheral,start ADV
		if (ble_scatternet_app_link_table[conn_id].role == GAP_LINK_ROLE_SLAVE) {
			printf("As peripheral, receive disconncect, start ADV\r\n");
#if defined(APP_LE_EXT_ADV_SCAN_SUPPORT) && APP_LE_EXT_ADV_SCAN_SUPPORT
			//uint8_t adv_handle = 0;
			//le_ext_adv_enable(1, &adv_handle);
#else
#if APP_PRIVACY_EN
			ble_scatternet_app_adv_start();
#else
			le_adv_start();
#endif
#endif
		}

		if (ble_scatternet_app_link_table[conn_id].role == GAP_LINK_ROLE_MASTER) {
			ble_scatternet_central_app_max_links --;
		} else if (ble_scatternet_app_link_table[conn_id].role == GAP_LINK_ROLE_SLAVE) {
			ble_scatternet_peripheral_app_max_links --;
		}

		memset(&ble_scatternet_app_link_table[conn_id], 0, sizeof(T_APP_LINK));
	}
	break;

	case GAP_CONN_STATE_CONNECTED: {
		le_get_conn_addr(conn_id, ble_scatternet_app_link_table[conn_id].bd_addr,
						 (void *)&ble_scatternet_app_link_table[conn_id].bd_type);
		//get device role
		if (le_get_conn_info(conn_id, &conn_info)) {
			ble_scatternet_app_link_table[conn_id].role = conn_info.role;
			if (ble_scatternet_app_link_table[conn_id].role == GAP_LINK_ROLE_MASTER) {
				ble_scatternet_central_app_max_links ++;
			} else if (ble_scatternet_app_link_table[conn_id].role == GAP_LINK_ROLE_SLAVE) {
				ble_scatternet_peripheral_app_max_links ++;
			}
		}

		printf("Connected success conn_id %d\r\n", conn_id);
		////print bt address type
		uint8_t local_bd_type;
		//uint8_t features[8];
		uint8_t remote_bd_type;
		le_get_conn_param(GAP_PARAM_CONN_LOCAL_BD_TYPE, &local_bd_type, conn_id);
		le_get_conn_param(GAP_PARAM_CONN_BD_ADDR_TYPE, &remote_bd_type, conn_id);
		APP_PRINT_INFO3("GAP_CONN_STATE_CONNECTED: conn_id %d, local_bd_type %d, remote_bd_type %d\n",
						conn_id, local_bd_type, remote_bd_type);
		printf("GAP_CONN_STATE_CONNECTED: conn_id %d, local_bd_type %d, remote_bd_type %d\r\n",
			   conn_id, local_bd_type, remote_bd_type);

#if F_BT_LE_5_0_SET_PHY_SUPPORT
		{
			uint8_t tx_phy;
			uint8_t rx_phy;
			le_get_conn_param(GAP_PARAM_CONN_RX_PHY_TYPE, &rx_phy, conn_id);
			le_get_conn_param(GAP_PARAM_CONN_TX_PHY_TYPE, &tx_phy, conn_id);
			APP_PRINT_INFO2("GAP_CONN_STATE_CONNECTED: tx_phy %d, rx_phy %d\n", tx_phy, rx_phy);
			printf("GAP_CONN_STATE_CONNECTED: tx_phy %d, rx_phy %d\r\n", tx_phy, rx_phy);
		}
#endif
	}
	break;

	default:
		break;
	}
}

/**
 * @brief    Handle msg GAP_MSG_LE_AUTHEN_STATE_CHANGE
 * @note     All the gap authentication state events are pre-handled in this function.
 *           Then the event handling function shall be called according to the new_state
 * @param[in] conn_id Connection ID
 * @param[in] new_state  New authentication state
 * @param[in] cause Use this cause when new_state is GAP_AUTHEN_STATE_COMPLETE
 * @return   void
 */
void ble_scatternet_app_handle_authen_state_evt(uint8_t conn_id, uint8_t new_state, uint16_t cause)
{
	APP_PRINT_INFO2("ble_scatternet_app_handle_authen_state_evt:conn_id %d, cause 0x%x", conn_id, cause);

	switch (new_state) {
	case GAP_AUTHEN_STATE_STARTED: {
		APP_PRINT_INFO0("ble_scatternet_app_handle_authen_state_evt: GAP_AUTHEN_STATE_STARTED");
	}
	break;

	case GAP_AUTHEN_STATE_COMPLETE: {
		if (cause == GAP_SUCCESS) {
			printf("Pair success\r\n");
			APP_PRINT_INFO0("ble_scatternet_app_handle_authen_state_evt: GAP_AUTHEN_STATE_COMPLETE pair success");
		} else {
			printf("Pair failed: cause 0x%x\r\n", cause);
			APP_PRINT_INFO0("ble_scatternet_app_handle_authen_state_evt: GAP_AUTHEN_STATE_COMPLETE pair failed");
		}
	}
	break;

	default: {
		APP_PRINT_ERROR1("ble_scatternet_app_handle_authen_state_evt: unknown newstate %d", new_state);
	}
	break;
	}
}

/**
 * @brief    Handle msg GAP_MSG_LE_CONN_MTU_INFO
 * @note     This msg is used to inform APP that exchange mtu procedure is completed.
 * @param[in] conn_id Connection ID
 * @param[in] mtu_size  New mtu size
 * @return   void
 */
void ble_scatternet_app_handle_conn_mtu_info_evt(uint8_t conn_id, uint16_t mtu_size)
{
	APP_PRINT_INFO2("ble_scatternet_app_handle_conn_mtu_info_evt: conn_id %d, mtu_size %d", conn_id, mtu_size);
}

/**
 * @brief    Handle msg GAP_MSG_LE_CONN_PARAM_UPDATE
 * @note     All the connection parameter update change  events are pre-handled in this function.
 * @param[in] conn_id Connection ID
 * @param[in] status  New update state
 * @param[in] cause Use this cause when status is GAP_CONN_PARAM_UPDATE_STATUS_FAIL
 * @return   void
 */
void ble_scatternet_app_handle_conn_param_update_evt(uint8_t conn_id, uint8_t status, uint16_t cause)
{
	switch (status) {
	case GAP_CONN_PARAM_UPDATE_STATUS_SUCCESS: {
		uint16_t conn_interval;
		uint16_t conn_slave_latency;
		uint16_t conn_supervision_timeout;

		le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, conn_id);
		le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_slave_latency, conn_id);
		le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_supervision_timeout, conn_id);
		APP_PRINT_INFO4("ble_scatternet_app_handle_conn_param_update_evt update success:conn_id %d, conn_interval 0x%x, conn_slave_latency 0x%x, conn_supervision_timeout 0x%x",
						conn_id, conn_interval, conn_slave_latency, conn_supervision_timeout);
		printf("ble_scatternet_app_handle_conn_param_update_evt update success:conn_id %d, conn_interval 0x%x, conn_slave_latency 0x%x, conn_supervision_timeout 0x%x\r\n",
			   conn_id, conn_interval, conn_slave_latency, conn_supervision_timeout);
	}
	break;

	case GAP_CONN_PARAM_UPDATE_STATUS_FAIL: {
		APP_PRINT_ERROR2("ble_scatternet_app_handle_conn_param_update_evt update failed: conn_id %d, cause 0x%x",
						 conn_id, cause);
		printf("ble_scatternet_app_handle_conn_param_update_evt update failed: conn_id %d, cause 0x%x\r\n",
			   conn_id, cause);

	}
	break;

	case GAP_CONN_PARAM_UPDATE_STATUS_PENDING: {
		APP_PRINT_INFO1("ble_scatternet_app_handle_conn_param_update_evt update pending: conn_id %d", conn_id);
		printf("ble_scatternet_app_handle_conn_param_update_evt update pending: conn_id %d\r\n", conn_id);

	}
	break;

	default:
		break;
	}
}
#if defined(APP_LE_EXT_ADV_SCAN_SUPPORT) && APP_LE_EXT_ADV_SCAN_SUPPORT
extern ext_adv_info_t ext_adv_tbl[GAP_MAX_EXT_ADV_SETS];
void ble_scatternet_app_handle_ext_adv_state_evt(uint8_t adv_handle, T_GAP_EXT_ADV_STATE new_state, uint16_t cause)
{
	uint8_t idx = 0;
	T_GAP_EXT_ADV_STATE pre_state;

	for (idx = 0; idx < GAP_MAX_EXT_ADV_SETS; idx++) {
		if (ext_adv_tbl[idx].used &&
			ext_adv_tbl[idx].adv_handle == adv_handle) {
			pre_state = ext_adv_tbl[idx].ext_adv_state;
			ext_adv_tbl[idx].ext_adv_state = new_state;
			break;
		}
	}

	if (idx == GAP_MAX_EXT_ADV_SETS) {
		return;
	}

	APP_PRINT_INFO4("ble_scatternet_app_handle_ext_adv_state_evt: adv_handle 0x%x newState %d oldState %d cause 0x%x\r\n",
					adv_handle, new_state, pre_state, cause);

	if (EXT_ADV_STATE_IDLE == new_state) {
		if (EXT_ADV_STATE_START == pre_state) {
			printf("[ext_adv_state_evt]: Ext ADV[%d] started failed, cause: 0x%04x\r\n", adv_handle, cause);
		} else {
			if (cause == (HCI_ERR | HCI_ERR_OPERATION_CANCELLED_BY_HOST)) {
				printf("[ext_adv_state_evt]:Ext ADV[%d] stop by Host\r\n", adv_handle);
			} else if (cause == (HCI_ERR | HCI_ERR_DIRECTED_ADV_TIMEOUT) ||
					   cause == (HCI_ERR | HCI_ERR_LIMIT_REACHED)) {
				printf("[ext_adv_state_evt]:Ext ADV[%d] stop by duration\r\n", adv_handle);
			} else if (cause == 0) {
				printf("[ext_adv_state_evt]:Ext ADV[%d] stop by connection\r\n", adv_handle);
			}
		}

	} else {
		if (EXT_ADV_STATE_START == pre_state) {
			printf("[ext_adv_state_evt]: Ext ADV[%d] start success\r\n", adv_handle);
		} else if (EXT_ADV_STATE_STOP == pre_state) {
			printf("[ext_adv_state_evt]: Ext ADV[%d] stopped failed, cause: 0x%04x\r\n", adv_handle, cause);
		}
	}
}

void ble_scatternet_app_handle_ext_scan_state_evt(uint8_t pre_state, uint8_t new_state, uint16_t cause)
{
	APP_PRINT_INFO3("ble_scatternet_app_handle_ext_scan_state_evt: oldState %d, newState %d, cause 0x%x",
					pre_state, new_state, cause);
	//printf("ble_scatternet_app_handle_ext_scan_state_evt: oldState %d, newState %d, cause 0x%x\r\n",
					//pre_state, new_state, cause);
	if (pre_state != new_state) {
		if (GAP_SCAN_STATE_START == pre_state) {
			if (GAP_SCAN_STATE_IDLE == new_state) {
				printf("[ext_scan_state]Scan start fail\r\n");
			} else if (GAP_SCAN_STATE_SCANNING == new_state) {
				printf("[ext_scan_state]Scan start\r\n");
			}
		} else if (GAP_SCAN_STATE_STOP == pre_state) {
			if (GAP_SCAN_STATE_IDLE == new_state) {
				printf("[ext_scan_state]Scan stop\r\n");
			} else if (GAP_SCAN_STATE_SCANNING == new_state) {
				printf("[ext_scan_state]Scan stop fail\r\n");
			}
		} else if (GAP_SCAN_STATE_SCANNING == pre_state && GAP_SCAN_STATE_IDLE == new_state) {
			printf("[ext_scan_state]Scan stop due to timeout\r\n");
		}
	}
}
#endif

/**
 * @brief    All the BT GAP MSG are pre-handled in this function.
 * @note     Then the event handling function shall be called according to the
 *           subtype of T_IO_MSG
 * @param[in] p_gap_msg Pointer to GAP msg
 * @return   void
 */
void ble_scatternet_app_handle_gap_msg(T_IO_MSG *p_gap_msg)
{
	T_LE_GAP_MSG gap_msg;
	uint8_t conn_id;
	memcpy(&gap_msg, &p_gap_msg->u.param, sizeof(p_gap_msg->u.param));

	APP_PRINT_TRACE1("ble_scatternet_app_handle_gap_msg: subtype %d", p_gap_msg->subtype);
	switch (p_gap_msg->subtype) {
	case GAP_MSG_LE_DEV_STATE_CHANGE: {
		ble_scatternet_app_handle_dev_state_evt(gap_msg.msg_data.gap_dev_state_change.new_state,
												gap_msg.msg_data.gap_dev_state_change.cause);
	}
	break;

	case GAP_MSG_LE_CONN_STATE_CHANGE: {
		ble_scatternet_app_handle_conn_state_evt(gap_msg.msg_data.gap_conn_state_change.conn_id,
				(T_GAP_CONN_STATE)gap_msg.msg_data.gap_conn_state_change.new_state,
				gap_msg.msg_data.gap_conn_state_change.disc_cause);
	}
	break;

	case GAP_MSG_LE_CONN_MTU_INFO: {
		ble_scatternet_app_handle_conn_mtu_info_evt(gap_msg.msg_data.gap_conn_mtu_info.conn_id,
				gap_msg.msg_data.gap_conn_mtu_info.mtu_size);
	}
	break;

	case GAP_MSG_LE_CONN_PARAM_UPDATE: {
		ble_scatternet_app_handle_conn_param_update_evt(gap_msg.msg_data.gap_conn_param_update.conn_id,
				gap_msg.msg_data.gap_conn_param_update.status,
				gap_msg.msg_data.gap_conn_param_update.cause);
	}
	break;

	case GAP_MSG_LE_AUTHEN_STATE_CHANGE: {
		ble_scatternet_app_handle_authen_state_evt(gap_msg.msg_data.gap_authen_state.conn_id,
				gap_msg.msg_data.gap_authen_state.new_state,
				gap_msg.msg_data.gap_authen_state.status);
	}
	break;

	case GAP_MSG_LE_BOND_JUST_WORK: {
		conn_id = gap_msg.msg_data.gap_bond_just_work_conf.conn_id;
		le_bond_just_work_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
		APP_PRINT_INFO0("GAP_MSG_LE_BOND_JUST_WORK");
	}
	break;

	case GAP_MSG_LE_BOND_PASSKEY_DISPLAY: {
		uint32_t display_value = 0;
		conn_id = gap_msg.msg_data.gap_bond_passkey_display.conn_id;
		le_bond_get_display_key(conn_id, &display_value);
		APP_PRINT_INFO2("GAP_MSG_LE_BOND_PASSKEY_DISPLAY: conn_id %d, passkey %d",
						conn_id, display_value);
		le_bond_passkey_display_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
		printf("GAP_MSG_LE_BOND_PASSKEY_DISPLAY: conn_id %d, passkey %06d\r\n",
			   conn_id,
			   display_value);
	}
	break;

	case GAP_MSG_LE_BOND_USER_CONFIRMATION: {
		uint32_t display_value = 0;
		conn_id = gap_msg.msg_data.gap_bond_user_conf.conn_id;
		le_bond_get_display_key(conn_id, &display_value);
		APP_PRINT_INFO2("GAP_MSG_LE_BOND_USER_CONFIRMATION: conn_id %d, passkey %d",
						conn_id, display_value);
		printf("GAP_MSG_LE_BOND_USER_CONFIRMATION: conn_id %d, passkey %06d\r\n",
			   conn_id,
			   display_value);
		//le_bond_user_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
	}
	break;

	case GAP_MSG_LE_BOND_PASSKEY_INPUT: {
		//uint32_t passkey = 888888;
		conn_id = gap_msg.msg_data.gap_bond_passkey_input.conn_id;
		APP_PRINT_INFO1("GAP_MSG_LE_BOND_PASSKEY_INPUT: conn_id %d", conn_id);
		printf("GAP_MSG_LE_BOND_PASSKEY_INPUT: conn_id %d\r\n", conn_id);
		//le_bond_passkey_input_confirm(conn_id, passkey, GAP_CFM_CAUSE_ACCEPT);
	}
	break;
#if F_BT_LE_SMP_OOB_SUPPORT
	case GAP_MSG_LE_BOND_OOB_INPUT: {
		uint8_t oob_data[GAP_OOB_LEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		conn_id = gap_msg.msg_data.gap_bond_oob_input.conn_id;
		APP_PRINT_INFO1("GAP_MSG_LE_BOND_OOB_INPUT: conn_id %d", conn_id);
		le_bond_set_param(GAP_PARAM_BOND_OOB_DATA, GAP_OOB_LEN, oob_data);
		le_bond_oob_input_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
	}
	break;
#endif
#if defined(APP_LE_EXT_ADV_SCAN_SUPPORT) && APP_LE_EXT_ADV_SCAN_SUPPORT
	case GAP_MSG_LE_EXT_ADV_STATE_CHANGE: {
		ble_scatternet_app_handle_ext_adv_state_evt(gap_msg.msg_data.gap_ext_adv_state_change.adv_handle,
													(T_GAP_EXT_ADV_STATE)gap_msg.msg_data.gap_ext_adv_state_change.new_state,
													gap_msg.msg_data.gap_ext_adv_state_change.cause);
	}
	break;
#endif
	default:
		APP_PRINT_ERROR1("ble_scatternet_app_handle_gap_msg: unknown subtype %d", p_gap_msg->subtype);
		break;
	}
}
/** @} */ /* End of group CENTRAL_CLIENT_GAP_MSG */

/** @defgroup  CENTRAL_CLIENT_GAP_CALLBACK GAP Callback Event Handler
    * @brief Handle GAP callback event
    * @{
    */
/**
  * @brief Used to parse advertising data and scan response data
  * @param[in] scan_info point to scan information data.
  * @retval void
  */
void ble_scatternet_app_parse_scan_info(T_LE_SCAN_INFO *scan_info)
{
	uint8_t buffer[32];
	uint8_t pos = 0;

	while (pos < scan_info->data_len) {
		/* Length of the AD structure. */
		uint8_t length = scan_info->data[pos++];
		uint8_t type;

		if ((length > 0x01) && ((pos + length) <= 31)) {
			/* Copy the AD Data to buffer. */
			memcpy(buffer, scan_info->data + pos + 1, length - 1);
			/* AD Type, one octet. */
			type = scan_info->data[pos];

			APP_PRINT_TRACE2("ble_scatternet_app_parse_scan_info: AD Structure Info: AD type 0x%x, AD Data Length %d", type,
							 length - 1);
//            printf("ble_scatternet_app_parse_scan_info: AD Structure Info: AD type 0x%x, AD Data Length %d\r\n", type,
//                             length - 1);


			switch (type) {
			case GAP_ADTYPE_FLAGS: {
				/* (flags & 0x01) -- LE Limited Discoverable Mode */
				/* (flags & 0x02) -- LE General Discoverable Mode */
				/* (flags & 0x04) -- BR/EDR Not Supported */
				/* (flags & 0x08) -- Simultaneous LE and BR/EDR to Same Device Capable (Controller) */
				/* (flags & 0x10) -- Simultaneous LE and BR/EDR to Same Device Capable (Host) */
				uint8_t flags = scan_info->data[pos + 1];
				APP_PRINT_INFO1("GAP_ADTYPE_FLAGS: 0x%x", flags);
				printf("GAP_ADTYPE_FLAGS: 0x%x\r\n", flags);

			}
			break;

			case GAP_ADTYPE_16BIT_MORE:
			case GAP_ADTYPE_16BIT_COMPLETE:
			case GAP_ADTYPE_SERVICES_LIST_16BIT: {
				uint16_t *p_uuid = (uint16_t *)(buffer);
				uint8_t i = length - 1;

				while (i >= 2) {
					APP_PRINT_INFO1("GAP_ADTYPE_16BIT_XXX: 0x%x", *p_uuid);
					printf("GAP_ADTYPE_16BIT_XXX: 0x%x\r\n", *p_uuid);
					p_uuid ++;
					i -= 2;
				}
			}
			break;

			case GAP_ADTYPE_32BIT_MORE:
			case GAP_ADTYPE_32BIT_COMPLETE: {
				uint32_t *p_uuid = (uint32_t *)(buffer);
				uint8_t    i     = length - 1;

				while (i >= 4) {
					APP_PRINT_INFO1("GAP_ADTYPE_32BIT_XXX: 0x%x", *p_uuid);
					printf("GAP_ADTYPE_32BIT_XXX: 0x%x\r\n", (unsigned int)*p_uuid);
					p_uuid ++;

					i -= 4;
				}
			}
			break;

			case GAP_ADTYPE_128BIT_MORE:
			case GAP_ADTYPE_128BIT_COMPLETE:
			case GAP_ADTYPE_SERVICES_LIST_128BIT: {
				uint32_t *p_uuid = (uint32_t *)(buffer);
				APP_PRINT_INFO4("GAP_ADTYPE_128BIT_XXX: 0x%8.8x%8.8x%8.8x%8.8x",
								p_uuid[3], p_uuid[2], p_uuid[1], p_uuid[0]);
				printf("GAP_ADTYPE_128BIT_XXX: 0x%8.8x%8.8x%8.8x%8.8x\r\n",
					   (unsigned int)p_uuid[3], (unsigned int)p_uuid[2], (unsigned int)p_uuid[1], (unsigned int)p_uuid[0]);

			}
			break;

			case GAP_ADTYPE_LOCAL_NAME_SHORT:
			case GAP_ADTYPE_LOCAL_NAME_COMPLETE: {
				buffer[length - 1] = '\0';
				APP_PRINT_INFO1("GAP_ADTYPE_LOCAL_NAME_XXX: %s", TRACE_STRING(buffer));
				printf("GAP_ADTYPE_LOCAL_NAME_XXX: %s\r\n", buffer);

			}
			break;

			case GAP_ADTYPE_POWER_LEVEL: {
				APP_PRINT_INFO1("GAP_ADTYPE_POWER_LEVEL: 0x%x", scan_info->data[pos + 1]);
				printf("GAP_ADTYPE_POWER_LEVEL: 0x%x\r\n", scan_info->data[pos + 1]);

			}
			break;

			case GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE: {
				uint16_t *p_min = (uint16_t *)(buffer);
				uint16_t *p_max = p_min + 1;
				APP_PRINT_INFO2("GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE: 0x%x - 0x%x", *p_min,
								*p_max);
				printf("GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE: 0x%x - 0x%x\r\n", *p_min,
					   *p_max);

			}
			break;

			case GAP_ADTYPE_SERVICE_DATA: {
				uint16_t *p_uuid = (uint16_t *)(buffer);
				uint8_t data_len = length - 3;

				APP_PRINT_INFO3("GAP_ADTYPE_SERVICE_DATA: UUID 0x%x, len %d, data %b", *p_uuid,
								data_len, TRACE_BINARY(data_len, &buffer[2]));
				printf("GAP_ADTYPE_SERVICE_DATA: UUID 0x%x, len %d\r\n", *p_uuid,
					   data_len);

			}
			break;
			case GAP_ADTYPE_APPEARANCE: {
				uint16_t *p_appearance = (uint16_t *)(buffer);
				APP_PRINT_INFO1("GAP_ADTYPE_APPEARANCE: %d", *p_appearance);
				printf("GAP_ADTYPE_APPEARANCE: %d\r\n", *p_appearance);

			}
			break;

			case GAP_ADTYPE_MANUFACTURER_SPECIFIC: {
				uint8_t data_len = length - 3;
				uint16_t *p_company_id = (uint16_t *)(buffer);
				APP_PRINT_INFO3("GAP_ADTYPE_MANUFACTURER_SPECIFIC: company_id 0x%x, len %d, data %b",
								*p_company_id, data_len, TRACE_BINARY(data_len, &buffer[2]));
				printf("GAP_ADTYPE_MANUFACTURER_SPECIFIC: company_id 0x%x, len %d\r\n",
					   *p_company_id, data_len);

			}
			break;

			default: {
				uint8_t i = 0;

				for (i = 0; i < (length - 1); i++) {
					APP_PRINT_INFO1("  AD Data: Unhandled Data = 0x%x", scan_info->data[pos + i]);
//						printf("  AD Data: Unhandled Data = 0x%x\r\n", scan_info->data[pos + i]);

				}
			}
			break;
			}
		}

		pos += length;
	}
}
/**
  * @brief Callback for gap le to notify app
  * @param[in] cb_type callback msy type @ref GAP_LE_MSG_Types.
  * @param[in] p_cb_data point to callback data @ref T_LE_CB_DATA.
  * @retval result @ref T_APP_RESULT
  */
T_APP_RESULT ble_scatternet_app_gap_callback(uint8_t cb_type, void *p_cb_data)
{
	T_APP_RESULT result = APP_RESULT_SUCCESS;
	T_GAP_CAUSE cause = GAP_CAUSE_SUCCESS;
	T_LE_CB_DATA *p_data = (T_LE_CB_DATA *)p_cb_data;
	char adv_type[20];
	char remote_addr_type[10];

	switch (cb_type) {
	case GAP_MSG_LE_SCAN_INFO:
		APP_PRINT_INFO5("GAP_MSG_LE_SCAN_INFO:adv_type 0x%x, bd_addr %s, remote_addr_type %d, rssi %d, data_len %d",
						p_data->p_le_scan_info->adv_type,
						TRACE_BDADDR(p_data->p_le_scan_info->bd_addr),
						p_data->p_le_scan_info->remote_addr_type,
						p_data->p_le_scan_info->rssi,
						p_data->p_le_scan_info->data_len);
		/* If you want to parse the scan info, please reference function ble_central_app_parse_scan_info. */
		sprintf(adv_type, "%s", (p_data->p_le_scan_info->adv_type == GAP_ADV_EVT_TYPE_UNDIRECTED) ? "CON_UNDIRECT" :
				(p_data->p_le_scan_info->adv_type == GAP_ADV_EVT_TYPE_DIRECTED) ? "CON_DIRECT" :
				(p_data->p_le_scan_info->adv_type == GAP_ADV_EVT_TYPE_SCANNABLE) ? "SCANABLE_UNDIRCT" :
				(p_data->p_le_scan_info->adv_type == GAP_ADV_EVT_TYPE_NON_CONNECTABLE) ? "NON_CONNECTABLE" :
				(p_data->p_le_scan_info->adv_type == GAP_ADV_EVT_TYPE_SCAN_RSP) ? "SCAN_RSP" : "unknown");
		sprintf(remote_addr_type, "%s", (p_data->p_le_scan_info->remote_addr_type == GAP_REMOTE_ADDR_LE_PUBLIC) ? "public" :
				(p_data->p_le_scan_info->remote_addr_type == GAP_REMOTE_ADDR_LE_RANDOM) ? "random" : "unknown");

		printf("ADVType\t\t\t| AddrType\t|%s\t\t\t|rssi\r\n", "BT_Addr");
		printf("%s\t\t%s\t"BD_ADDR_FMT"\t%d\r\n", adv_type, remote_addr_type, BD_ADDR_ARG(p_data->p_le_scan_info->bd_addr),
			   p_data->p_le_scan_info->rssi);

		ble_scatternet_app_parse_scan_info(p_data->p_le_scan_info);
		break;

	case GAP_MSG_LE_CONN_UPDATE_IND:
		APP_PRINT_INFO5("GAP_MSG_LE_CONN_UPDATE_IND: conn_id %d, conn_interval_max 0x%x, conn_interval_min 0x%x, conn_latency 0x%x,supervision_timeout 0x%x",
						p_data->p_le_conn_update_ind->conn_id,
						p_data->p_le_conn_update_ind->conn_interval_max,
						p_data->p_le_conn_update_ind->conn_interval_min,
						p_data->p_le_conn_update_ind->conn_latency,
						p_data->p_le_conn_update_ind->supervision_timeout);
		printf("GAP_MSG_LE_CONN_UPDATE_IND: conn_id %d, conn_interval_max 0x%x, conn_interval_min 0x%x, conn_latency 0x%x, supervision_timeout 0x%x\r\n",
			   p_data->p_le_conn_update_ind->conn_id,
			   p_data->p_le_conn_update_ind->conn_interval_max,
			   p_data->p_le_conn_update_ind->conn_interval_min,
			   p_data->p_le_conn_update_ind->conn_latency,
			   p_data->p_le_conn_update_ind->supervision_timeout);
		/* if reject the proposed connection parameter from peer device, use APP_RESULT_REJECT. */
		result = APP_RESULT_ACCEPT;
		break;
#if F_BT_LE_5_0_SET_PHY_SUPPORT
	case GAP_MSG_LE_PHY_UPDATE_INFO:
		APP_PRINT_INFO4("GAP_MSG_LE_PHY_UPDATE_INFO:conn_id %d, cause 0x%x, rx_phy %d, tx_phy %d",
						p_data->p_le_phy_update_info->conn_id,
						p_data->p_le_phy_update_info->cause,
						p_data->p_le_phy_update_info->rx_phy,
						p_data->p_le_phy_update_info->tx_phy);
		printf("GAP_MSG_LE_PHY_UPDATE_INFO:conn_id %d, cause 0x%x, rx_phy %d, tx_phy %d\r\n",
			   p_data->p_le_phy_update_info->conn_id,
			   p_data->p_le_phy_update_info->cause,
			   p_data->p_le_phy_update_info->rx_phy,
			   p_data->p_le_phy_update_info->tx_phy);
		break;

	case GAP_MSG_LE_REMOTE_FEATS_INFO: {
		uint8_t  remote_feats[8];
		APP_PRINT_INFO3("GAP_MSG_LE_REMOTE_FEATS_INFO: conn id %d, cause 0x%x, remote_feats %b",
						p_data->p_le_remote_feats_info->conn_id,
						p_data->p_le_remote_feats_info->cause,
						TRACE_BINARY(8, p_data->p_le_remote_feats_info->remote_feats));
		if (p_data->p_le_remote_feats_info->cause == GAP_SUCCESS) {
			memcpy(remote_feats, p_data->p_le_remote_feats_info->remote_feats, 8);
			if (remote_feats[LE_SUPPORT_FEATURES_MASK_ARRAY_INDEX1] & LE_SUPPORT_FEATURES_LE_2M_MASK_BIT) {
				APP_PRINT_INFO0("GAP_MSG_LE_REMOTE_FEATS_INFO: support 2M");
				printf("GAP_MSG_LE_REMOTE_FEATS_INFO: support 2M\r\n");
			}
			if (remote_feats[LE_SUPPORT_FEATURES_MASK_ARRAY_INDEX1] & LE_SUPPORT_FEATURES_LE_CODED_PHY_MASK_BIT) {
				APP_PRINT_INFO0("GAP_MSG_LE_REMOTE_FEATS_INFO: support CODED");
				printf("GAP_MSG_LE_REMOTE_FEATS_INFO: support CODED\r\n");
			}
		}
	}
	break;
#endif

	case GAP_MSG_LE_MODIFY_WHITE_LIST:
		APP_PRINT_INFO2("GAP_MSG_LE_MODIFY_WHITE_LIST: operation  0x%x, cause 0x%x",
						p_data->p_le_modify_white_list_rsp->operation,
						p_data->p_le_modify_white_list_rsp->cause);
		printf("GAP_MSG_LE_MODIFY_WHITE_LIST: operation  0x%x, cause 0x%x\r\n",
			   p_data->p_le_modify_white_list_rsp->operation,
			   p_data->p_le_modify_white_list_rsp->cause);
		break;
#if defined(APP_LE_EXT_ADV_SCAN_SUPPORT) && APP_LE_EXT_ADV_SCAN_SUPPORT
	case GAP_MSG_LE_EXT_ADV_START_SETTING:
		APP_PRINT_INFO3("GAP_MSG_LE_EXT_ADV_START_SETTING:cause 0x%x, flag 0x%x, adv_handle 0x%x",
						p_data->p_le_ext_adv_start_setting_rsp->cause,
						p_data->p_le_ext_adv_start_setting_rsp->flag,
						p_data->p_le_ext_adv_start_setting_rsp->adv_handle);
		if (GAP_SUCCESS != p_data->p_le_ext_adv_start_setting_rsp->cause) {
			random_update_flag = 0;
			printf("GAP_MSG_LE_EXT_ADV_START_SETTING:cause 0x%x, flag 0x%x, adv_handle 0x%x\r\n",
					p_data->p_le_ext_adv_start_setting_rsp->cause,
					p_data->p_le_ext_adv_start_setting_rsp->flag,
					p_data->p_le_ext_adv_start_setting_rsp->adv_handle);
		} else {
			if (0 == random_update_flag) {
				cause = le_ext_adv_enable(1, &p_data->p_le_ext_adv_start_setting_rsp->adv_handle);
				if (cause) {
					printf("le_ext_adv_enable cause = 0x%x \r\n", cause);
				}
			} else {
				random_update_flag = 0;
			}
		}
		break;
	case GAP_MSG_LE_EXT_ADV_REMOVE_SET:
		APP_PRINT_INFO2("GAP_MSG_LE_EXT_ADV_REMOVE_SET:cause 0x%x, adv_handle 0x%x",
						p_data->p_le_ext_adv_remove_set_rsp->cause,
						p_data->p_le_ext_adv_remove_set_rsp->adv_handle);

		if (GAP_SUCCESS == p_data->p_le_ext_adv_remove_set_rsp->cause) {
			for (uint8_t idx = 0; idx < GAP_MAX_EXT_ADV_SETS; idx++) {
				if (ext_adv_tbl[idx].used &&  ext_adv_tbl[idx].adv_handle == p_data->p_le_ext_adv_remove_set_rsp->adv_handle) {
					ext_adv_tbl[idx].used = false;
					ext_adv_tbl[idx].adv_handle = GAP_INVALID_ADV_HANDLE;
					ext_adv_tbl[idx].ext_adv_state = EXT_ADV_STATE_IDLE;
				}
			}
		} else {
			printf("GAP_MSG_LE_EXT_ADV_REMOVE_SET:cause 0x%x, adv_handle 0x%x\r\n",
					p_data->p_le_ext_adv_remove_set_rsp->cause,
				    p_data->p_le_ext_adv_remove_set_rsp->adv_handle);
		}
		break;
	case GAP_MSG_LE_EXT_ADV_CLEAR_SET:
		APP_PRINT_INFO1("GAP_MSG_LE_EXT_ADV_CLEAR_SET:cause 0x%x",
						p_data->p_le_ext_adv_clear_set_rsp->cause);

		if (GAP_SUCCESS == p_data->p_le_ext_adv_clear_set_rsp->cause) {
			for (uint8_t idx = 0; idx < GAP_MAX_EXT_ADV_SETS; idx++) {
				if (ext_adv_tbl[idx].used) {
					ext_adv_tbl[idx].used = false;
					ext_adv_tbl[idx].adv_handle = GAP_INVALID_ADV_HANDLE;
					ext_adv_tbl[idx].ext_adv_state = EXT_ADV_STATE_IDLE;
				}
			}
		} else {
			printf("GAP_MSG_LE_EXT_ADV_CLEAR_SET:cause 0x%x\r\n",
					p_data->p_le_ext_adv_clear_set_rsp->cause);
		}
		break;
	case GAP_MSG_LE_EXT_ADV_ENABLE:
		APP_PRINT_INFO1("GAP_MSG_LE_EXT_ADV_ENABLE:cause 0x%x",
						p_data->le_cause.cause);
		printf("GAP_MSG_LE_EXT_ADV_ENABLE:cause 0x%x\r\n",
						p_data->le_cause.cause);
		break;
	case GAP_MSG_LE_EXT_ADV_DISABLE:
		APP_PRINT_INFO1("GAP_MSG_LE_EXT_ADV_DISABLE:cause 0x%x",
						p_data->le_cause.cause);
		printf("GAP_MSG_LE_EXT_ADV_DISABLE:cause 0x%x\r\n",
						p_data->le_cause.cause);
		break;
	case GAP_MSG_LE_EXT_ADV_STATE_CHANGE_INFO:
		{
			ble_scatternet_app_handle_ext_adv_state_evt(p_data->p_le_ext_adv_state_change_info->adv_handle,
														(T_GAP_EXT_ADV_STATE)p_data->p_le_ext_adv_state_change_info->state,
														p_data->p_le_ext_adv_state_change_info->cause);
		}
		break;
	case GAP_MSG_LE_EXT_SCAN_STATE_CHANGE_INFO:
        {
			uint8_t pre_scan_state = ble_scatternet_gap_dev_state.gap_scan_state;
			ble_scatternet_gap_dev_state.gap_scan_state = p_data->p_le_ext_scan_state_change_info->state;
            ble_scatternet_app_handle_ext_scan_state_evt(pre_scan_state,
														p_data->p_le_ext_scan_state_change_info->state,
                                          				p_data->p_le_ext_scan_state_change_info->cause);
        }
        	break;
	case GAP_MSG_LE_EXT_ADV_REPORT_INFO:
		APP_PRINT_INFO6("GAP_MSG_LE_EXT_ADV_REPORT_INFO:connectable %d, scannable %d, direct %d, scan response %d, legacy %d, data status 0x%x",
						p_data->p_le_ext_adv_report_info->event_type & GAP_EXT_ADV_REPORT_BIT_CONNECTABLE_ADV,
						p_data->p_le_ext_adv_report_info->event_type & GAP_EXT_ADV_REPORT_BIT_SCANNABLE_ADV,
						p_data->p_le_ext_adv_report_info->event_type & GAP_EXT_ADV_REPORT_BIT_DIRECTED_ADV,
						p_data->p_le_ext_adv_report_info->event_type & GAP_EXT_ADV_REPORT_BIT_SCAN_RESPONSE,
						p_data->p_le_ext_adv_report_info->event_type & GAP_EXT_ADV_REPORT_BIT_USE_LEGACY_ADV,
						p_data->p_le_ext_adv_report_info->data_status);
		APP_PRINT_INFO5("GAP_MSG_LE_EXT_ADV_REPORT_INFO:event_type 0x%x, bd_addr %s, addr_type %d, rssi %d, data_len %d",
						p_data->p_le_ext_adv_report_info->event_type,
						TRACE_BDADDR(p_data->p_le_ext_adv_report_info->bd_addr),
						p_data->p_le_ext_adv_report_info->addr_type,
						p_data->p_le_ext_adv_report_info->rssi,
						p_data->p_le_ext_adv_report_info->data_len);
		if ((p_data->p_le_ext_adv_report_info->event_type & GAP_EXT_ADV_REPORT_BIT_USE_LEGACY_ADV) == 0) {
		APP_PRINT_INFO5("GAP_MSG_LE_EXT_ADV_REPORT_INFO:primary_phy %d, secondary_phy %d, adv_sid %d, tx_power %d, peri_adv_interval %d",
						p_data->p_le_ext_adv_report_info->primary_phy,
						p_data->p_le_ext_adv_report_info->secondary_phy,
						p_data->p_le_ext_adv_report_info->adv_sid,
						p_data->p_le_ext_adv_report_info->tx_power,
						p_data->p_le_ext_adv_report_info->peri_adv_interval);
		}
		if (p_data->p_le_ext_adv_report_info->event_type & GAP_EXT_ADV_REPORT_BIT_DIRECTED_ADV) {
		APP_PRINT_INFO2("GAP_MSG_LE_EXT_ADV_REPORT_INFO:direct_addr_type 0x%x, direct_addr %s",
						p_data->p_le_ext_adv_report_info->direct_addr_type,
						TRACE_BDADDR(p_data->p_le_ext_adv_report_info->direct_addr));
		}

		#if 0
		/* If you want to parse the ext scan info, please reference function ble_central_app_parse_ext_scan_info. */
		sprintf(ext_adv_type, "%s", (p_data->p_le_ext_adv_report_info->event_type == GAP_EXT_ADV_REPORT_BIT_CONNECTABLE_ADV) ? "CON_UNDIRECT" :
				(p_data->p_le_ext_adv_report_info->event_type == GAP_EXT_ADV_REPORT_BIT_SCANNABLE_ADV) ? "SCANABLE_UNDIRECT" :
				(p_data->p_le_ext_adv_report_info->event_type == GAP_EXT_ADV_REPORT_BIT_DIRECTED_ADV) ? "DIRECT" :
				(p_data->p_le_ext_adv_report_info->event_type == GAP_EXT_ADV_REPORT_BIT_SCAN_RESPONSE) ? "SCAN_RSP" :
				(p_data->p_le_ext_adv_report_info->event_type == GAP_EXT_ADV_REPORT_BIT_USE_LEGACY_ADV) ? "LEGACY_ADV" : "unknown");
		sprintf(addr_type, "%s", (p_data->p_le_ext_adv_report_info->addr_type == GAP_REMOTE_ADDR_LE_PUBLIC) ? "public" :
				(p_data->p_le_ext_adv_report_info->addr_type == GAP_REMOTE_ADDR_LE_RANDOM) ? "random" : "unknown");
		printf("ADVType\t\t\t| AddrType\t|%s\t\t\t|rssi\r\n", "BT_Addr");
		printf("%s\t\t%s\t"BD_ADDR_FMT"\t%d\r\n", ext_adv_type, addr_type, BD_ADDR_ARG(p_data->p_le_ext_adv_report_info->bd_addr),
			   p_data->p_le_ext_adv_report_info->rssi);
		ble_central_app_parse_ext_scan_info(p_data->p_le_ext_adv_report_info);
		#endif
		printf("Ext Scan info, [Device]:"BD_ADDR_FMT", AD evt type: 0x%x, RSSI: %d, PHY: 0x%x, TxPower: %d, Len: %d\r\n",
			   BD_ADDR_ARG(p_data->p_le_ext_adv_report_info->bd_addr), p_data->p_le_ext_adv_report_info->event_type, p_data->p_le_ext_adv_report_info->rssi,
			   (p_data->p_le_ext_adv_report_info->primary_phy << 4) | p_data->p_le_ext_adv_report_info->secondary_phy,
			   p_data->p_le_ext_adv_report_info->tx_power, p_data->p_le_ext_adv_report_info->data_len);
		break;
#endif
#if APP_PRIVACY_EN
	case GAP_MSG_LE_BOND_MODIFY_INFO:
		APP_PRINT_INFO1("GAP_MSG_LE_BOND_MODIFY_INFO: type 0x%x",
						p_data->p_le_bond_modify_info->type);
		privacy_handle_bond_modify_msg(p_data->p_le_bond_modify_info->type,
									   p_data->p_le_bond_modify_info->p_entry, true);
		break;
#endif

	default:
		APP_PRINT_ERROR1("ble_scatternet_app_gap_callback: unhandled cb_type 0x%x", cb_type);
		break;
	}
	return result;
}
/** @} */ /* End of group CENTRAL_CLIENT_GAP_CALLBACK */
#if F_BT_GAPS_CHAR_WRITEABLE
/** @defgroup  SCATTERNET_GAPS_WRITE GAP Service Callback Handler
    * @brief Use @ref F_BT_GAPS_CHAR_WRITEABLE to open
    * @{
    */
/**
 * @brief    All the BT GAP service callback events are handled in this function
 * @param[in] service_id  Profile service ID
 * @param[in] p_para      Pointer to callback data
 * @return   Indicates the function call is successful or not
 * @retval   result @ref T_APP_RESULT
 */
T_APP_RESULT ble_scatternet_gap_service_callback(T_SERVER_ID service_id, void *p_para)
{
	(void) service_id;
	T_APP_RESULT  result = APP_RESULT_SUCCESS;
	T_GAPS_CALLBACK_DATA *p_gap_data = (T_GAPS_CALLBACK_DATA *)p_para;
	APP_PRINT_INFO2("ble_scatternet_gap_service_callback: conn_id = %d msg_type = %d\n", p_gap_data->conn_id,
					p_gap_data->msg_type);
	APP_PRINT_INFO2("ble_scatternet_gap_service_callback: len = 0x%x,opcode = %d\n", p_gap_data->msg_data.len,
					p_gap_data->msg_data.opcode);
	if (p_gap_data->msg_type == SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE) {
		switch (p_gap_data->msg_data.opcode) {
		case GAPS_WRITE_DEVICE_NAME: {
			T_LOCAL_NAME device_name;
			memcpy(device_name.local_name, p_gap_data->msg_data.p_value, p_gap_data->msg_data.len);
			device_name.local_name[p_gap_data->msg_data.len] = 0;
			//printf("GAPS_WRITE_DEVICE_NAME:device_name = %s\r\n",device_name.local_name);
			flash_save_local_name(&device_name);
		}
		break;

		case GAPS_WRITE_APPEARANCE: {
			uint16_t appearance_val;
			T_LOCAL_APPEARANCE appearance;

			LE_ARRAY_TO_UINT16(appearance_val, p_gap_data->msg_data.p_value);
			appearance.local_appearance = appearance_val;
			//printf("GAPS_WRITE_APPEARANCE:appearance = %s\r\n",appearance.local_appearance);
			flash_save_local_appearance(&appearance);
		}
		break;
		default:
			APP_PRINT_ERROR1("ble_scatternet_gap_service_callback: unhandled msg_data.opcode 0x%x", p_gap_data->msg_data.opcode);
			//printf("ble_scatternet_gap_service_callback: unhandled msg_data.opcode 0x%x\r\n", p_gap_data->msg_data.opcode);
			break;
		}
	}
	return result;
}
/** @} */
#endif


/** @defgroup  GCS_CLIIENT_CALLBACK GCS Client Callback Event Handler
    * @brief Handle profile client callback event
    * @{
    */
void ble_scatternet_gcs_handle_discovery_result(uint8_t conn_id, T_GCS_DISCOVERY_RESULT discov_result)
{
	uint16_t i;
	T_GCS_DISCOV_RESULT *p_result_table;
	uint16_t    properties;
	switch (discov_result.discov_type) {
	case GCS_ALL_PRIMARY_SRV_DISCOV:
		APP_PRINT_INFO2("conn_id %d, GCS_ALL_PRIMARY_SRV_DISCOV, is_success %d",
						conn_id, discov_result.is_success);
		for (i = 0; i < discov_result.result_num; i++) {
			p_result_table = &(discov_result.p_result_table[i]);
			switch (p_result_table->result_type) {
			case DISC_RESULT_ALL_SRV_UUID16:
				APP_PRINT_INFO4("ALL SRV UUID16[%d]: service range: 0x%x-0x%x, uuid16 0x%x",
								i, p_result_table->result_data.srv_uuid16_disc_data.att_handle,
								p_result_table->result_data.srv_uuid16_disc_data.end_group_handle,
								p_result_table->result_data.srv_uuid16_disc_data.uuid16);
				printf("ALL SRV UUID16[%d]: service range: 0x%x-0x%x, uuid16 0x%x\r\n",
					   i, p_result_table->result_data.srv_uuid16_disc_data.att_handle,
					   p_result_table->result_data.srv_uuid16_disc_data.end_group_handle,
					   p_result_table->result_data.srv_uuid16_disc_data.uuid16);
				break;
			case DISC_RESULT_ALL_SRV_UUID128:
				APP_PRINT_INFO4("ALL SRV UUID128[%d]: service range: 0x%x-0x%x, service=<%b>",
								i, p_result_table->result_data.srv_uuid128_disc_data.att_handle,
								p_result_table->result_data.srv_uuid128_disc_data.end_group_handle,
								TRACE_BINARY(16, p_result_table->result_data.srv_uuid128_disc_data.uuid128));
				printf("ALL SRV UUID128[%d]: service range: 0x%x-0x%x, service="UUID_128_FORMAT"\r\n",
					   i, p_result_table->result_data.srv_uuid128_disc_data.att_handle,
					   p_result_table->result_data.srv_uuid128_disc_data.end_group_handle,
					   UUID_128(p_result_table->result_data.srv_uuid128_disc_data.uuid128));

				break;

			default:
				APP_PRINT_ERROR0("Invalid Discovery Result Type!");
				printf("Invalid Discovery Result Type!\r\n");
				break;
			}
		}
		break;

	case GCS_BY_UUID128_SRV_DISCOV:
		APP_PRINT_INFO2("conn_id %d, GCS_BY_UUID128_SRV_DISCOV, is_success %d",
						conn_id, discov_result.is_success);
		printf("conn_id %d, GCS_BY_UUID128_SRV_DISCOV, is_success %d\r\n",
			   conn_id, discov_result.is_success);

		for (i = 0; i < discov_result.result_num; i++) {
			p_result_table = &(discov_result.p_result_table[i]);
			switch (p_result_table->result_type) {
			case DISC_RESULT_SRV_DATA:
				APP_PRINT_INFO3("SRV DATA[%d]: service range: 0x%x-0x%x",
								i, p_result_table->result_data.srv_disc_data.att_handle,
								p_result_table->result_data.srv_disc_data.end_group_handle);
				printf("SRV DATA[%d]: service range: 0x%x-0x%x\r\n",
					   i, p_result_table->result_data.srv_disc_data.att_handle,
					   p_result_table->result_data.srv_disc_data.end_group_handle);

				break;

			default:
				APP_PRINT_ERROR0("Invalid Discovery Result Type!");
				printf("Invalid Discovery Result Type!\r\n");
				break;
			}
		}
		break;

	case GCS_BY_UUID_SRV_DISCOV:
		APP_PRINT_INFO2("conn_id %d, GCS_BY_UUID_SRV_DISCOV, is_success %d",
						conn_id, discov_result.is_success);
		printf("conn_id %d, GCS_BY_UUID_SRV_DISCOV, is_success %d\r\n",
			   conn_id, discov_result.is_success);

		for (i = 0; i < discov_result.result_num; i++) {
			p_result_table = &(discov_result.p_result_table[i]);
			switch (p_result_table->result_type) {
			case DISC_RESULT_SRV_DATA:
				APP_PRINT_INFO3("SRV DATA[%d]: service range: 0x%x-0x%x",
								i, p_result_table->result_data.srv_disc_data.att_handle,
								p_result_table->result_data.srv_disc_data.end_group_handle);
				printf("SRV DATA[%d]: service range: 0x%x-0x%x\r\n",
					   i, p_result_table->result_data.srv_disc_data.att_handle,
					   p_result_table->result_data.srv_disc_data.end_group_handle);

				break;

			default:
				APP_PRINT_ERROR0("Invalid Discovery Result Type!");
				printf("Invalid Discovery Result Type!\r\n");
				break;
			}
		}
		break;

	case GCS_ALL_CHAR_DISCOV:
		APP_PRINT_INFO2("conn_id %d, GCS_ALL_CHAR_DISCOV, is_success %d",
						conn_id, discov_result.is_success);
		printf("conn_id %d, GCS_ALL_CHAR_DISCOV, is_success %d\r\n",
			   conn_id, discov_result.is_success);

		for (i = 0; i < discov_result.result_num; i++) {
			p_result_table = &(discov_result.p_result_table[i]);
			switch (p_result_table->result_type) {
			case DISC_RESULT_CHAR_UUID16:
				properties = p_result_table->result_data.char_uuid16_disc_data.properties;
				APP_PRINT_INFO5("CHAR UUID16[%d]: decl_handle 0x%x, properties 0x%x, value_handle 0x%x, uuid16 0x%x",
								i, p_result_table->result_data.char_uuid16_disc_data.decl_handle,
								p_result_table->result_data.char_uuid16_disc_data.properties,
								p_result_table->result_data.char_uuid16_disc_data.value_handle,
								p_result_table->result_data.char_uuid16_disc_data.uuid16);
				APP_PRINT_INFO5("properties:indicate %d, read %d, write cmd %d, write %d, notify %d\r\n",
								properties & GATT_CHAR_PROP_INDICATE,
								properties & GATT_CHAR_PROP_READ,
								properties & GATT_CHAR_PROP_WRITE_NO_RSP,
								properties & GATT_CHAR_PROP_WRITE,
								properties & GATT_CHAR_PROP_NOTIFY);
				printf("CHAR UUID16[%d]: decl_handle 0x%x, properties 0x%x, value_handle 0x%x, uuid16 0x%x\r\n",
					   i, p_result_table->result_data.char_uuid16_disc_data.decl_handle,
					   p_result_table->result_data.char_uuid16_disc_data.properties,
					   p_result_table->result_data.char_uuid16_disc_data.value_handle,
					   p_result_table->result_data.char_uuid16_disc_data.uuid16);
				printf("properties:indicate %d, read %d, write cmd %d, write %d, notify %d\r\n",
					   properties & GATT_CHAR_PROP_INDICATE,
					   properties & GATT_CHAR_PROP_READ,
					   properties & GATT_CHAR_PROP_WRITE_NO_RSP,
					   properties & GATT_CHAR_PROP_WRITE,
					   properties & GATT_CHAR_PROP_NOTIFY);


				break;

			case DISC_RESULT_CHAR_UUID128:
				properties = p_result_table->result_data.char_uuid128_disc_data.properties;
				APP_PRINT_INFO5("CHAR UUID128[%d]:  decl hndl=0x%x, prop=0x%x, value hndl=0x%x, uuid128=<%b>",
								i, p_result_table->result_data.char_uuid128_disc_data.decl_handle,
								p_result_table->result_data.char_uuid128_disc_data.properties,
								p_result_table->result_data.char_uuid128_disc_data.value_handle,
								TRACE_BINARY(16, p_result_table->result_data.char_uuid128_disc_data.uuid128));
				APP_PRINT_INFO5("properties:indicate %d, read %d, write cmd %d, write %d, notify %d",
								properties & GATT_CHAR_PROP_INDICATE,
								properties & GATT_CHAR_PROP_READ,
								properties & GATT_CHAR_PROP_WRITE_NO_RSP,
								properties & GATT_CHAR_PROP_WRITE,
								properties & GATT_CHAR_PROP_NOTIFY
							   );
				printf("CHAR UUID128[%d]:  decl hndl=0x%x, prop=0x%x, value hndl=0x%x, uuid128="UUID_128_FORMAT"\r\n",
					   i, p_result_table->result_data.char_uuid128_disc_data.decl_handle,
					   p_result_table->result_data.char_uuid128_disc_data.properties,
					   p_result_table->result_data.char_uuid128_disc_data.value_handle,
					   UUID_128(p_result_table->result_data.char_uuid128_disc_data.uuid128));
				printf("properties:indicate %d, read %d, write cmd %d, write %d, notify %d\r\n",
					   properties & GATT_CHAR_PROP_INDICATE,
					   properties & GATT_CHAR_PROP_READ,
					   properties & GATT_CHAR_PROP_WRITE_NO_RSP,
					   properties & GATT_CHAR_PROP_WRITE,
					   properties & GATT_CHAR_PROP_NOTIFY
					  );

				break;
			default:
				APP_PRINT_ERROR0("Invalid Discovery Result Type!");
				printf("Invalid Discovery Result Type!\r\n");
				break;
			}
		}
		break;

	case GCS_BY_UUID_CHAR_DISCOV:
		APP_PRINT_INFO2("conn_id %d, GCS_BY_UUID_CHAR_DISCOV, is_success %d",
						conn_id, discov_result.is_success);
		printf("conn_id %d, GCS_BY_UUID_CHAR_DISCOV, is_success %d\r\n",
			   conn_id, discov_result.is_success);

		for (i = 0; i < discov_result.result_num; i++) {
			p_result_table = &(discov_result.p_result_table[i]);
			switch (p_result_table->result_type) {
			case DISC_RESULT_BY_UUID16_CHAR:
				properties = p_result_table->result_data.char_uuid16_disc_data.properties;
				APP_PRINT_INFO5("UUID16 CHAR[%d]: Characteristics by uuid16, decl hndl=0x%x, prop=0x%x, value hndl=0x%x, uuid16=<0x%x>",
								i, p_result_table->result_data.char_uuid16_disc_data.decl_handle,
								p_result_table->result_data.char_uuid16_disc_data.properties,
								p_result_table->result_data.char_uuid16_disc_data.value_handle,
								p_result_table->result_data.char_uuid16_disc_data.uuid16);
				APP_PRINT_INFO5("properties:indicate %d, read %d, write cmd %d, write %d, notify %d",
								properties & GATT_CHAR_PROP_INDICATE,
								properties & GATT_CHAR_PROP_READ,
								properties & GATT_CHAR_PROP_WRITE_NO_RSP,
								properties & GATT_CHAR_PROP_WRITE,
								properties & GATT_CHAR_PROP_NOTIFY
							   );
				printf("UUID16 CHAR[%d]: Characteristics by uuid16, decl hndl=0x%x, prop=0x%x, value hndl=0x%x, uuid16=<0x%x>\r\n",
					   i, p_result_table->result_data.char_uuid16_disc_data.decl_handle,
					   p_result_table->result_data.char_uuid16_disc_data.properties,
					   p_result_table->result_data.char_uuid16_disc_data.value_handle,
					   p_result_table->result_data.char_uuid16_disc_data.uuid16);
				printf("properties:indicate %d, read %d, write cmd %d, write %d, notify %d\r\n",
					   properties & GATT_CHAR_PROP_INDICATE,
					   properties & GATT_CHAR_PROP_READ,
					   properties & GATT_CHAR_PROP_WRITE_NO_RSP,
					   properties & GATT_CHAR_PROP_WRITE,
					   properties & GATT_CHAR_PROP_NOTIFY
					  );

				break;

			default:
				APP_PRINT_ERROR0("Invalid Discovery Result Type!");
				printf("Invalid Discovery Result Type!\r\n");
				break;
			}
		}
		break;

	case GCS_BY_UUID128_CHAR_DISCOV:
		APP_PRINT_INFO2("conn_id %d, GCS_BY_UUID128_CHAR_DISCOV, is_success %d",
						conn_id, discov_result.is_success);
		printf("conn_id %d, GCS_BY_UUID128_CHAR_DISCOV, is_success %d\r\n",
			   conn_id, discov_result.is_success);

		for (i = 0; i < discov_result.result_num; i++) {
			p_result_table = &(discov_result.p_result_table[i]);
			switch (p_result_table->result_type) {
			case DISC_RESULT_BY_UUID128_CHAR:
				properties = p_result_table->result_data.char_uuid128_disc_data.properties;
				APP_PRINT_INFO5("UUID128 CHAR[%d]: Characteristics by uuid128, decl hndl=0x%x, prop=0x%x, value hndl=0x%x, uuid128=<%b>",
								i, p_result_table->result_data.char_uuid128_disc_data.decl_handle,
								p_result_table->result_data.char_uuid128_disc_data.properties,
								p_result_table->result_data.char_uuid128_disc_data.value_handle,
								TRACE_BINARY(16, p_result_table->result_data.char_uuid128_disc_data.uuid128));
				APP_PRINT_INFO5("properties:indicate %d, read %d, write cmd %d, write %d, notify %d",
								properties & GATT_CHAR_PROP_INDICATE,
								properties & GATT_CHAR_PROP_READ,
								properties & GATT_CHAR_PROP_WRITE_NO_RSP,
								properties & GATT_CHAR_PROP_WRITE,
								properties & GATT_CHAR_PROP_NOTIFY
							   );
				printf("UUID128 CHAR[%d]: Characteristics by uuid128, decl hndl=0x%x, prop=0x%x, value hndl=0x%x, uuid128="UUID_128_FORMAT"\r\n",
					   i, p_result_table->result_data.char_uuid128_disc_data.decl_handle,
					   p_result_table->result_data.char_uuid128_disc_data.properties,
					   p_result_table->result_data.char_uuid128_disc_data.value_handle,
					   UUID_128(p_result_table->result_data.char_uuid128_disc_data.uuid128));
				printf("properties:indicate %d, read %d, write cmd %d, write %d, notify %d\r\n",
					   properties & GATT_CHAR_PROP_INDICATE,
					   properties & GATT_CHAR_PROP_READ,
					   properties & GATT_CHAR_PROP_WRITE_NO_RSP,
					   properties & GATT_CHAR_PROP_WRITE,
					   properties & GATT_CHAR_PROP_NOTIFY
					  );

				break;

			default:
				APP_PRINT_ERROR0("Invalid Discovery Result Type!");
				printf("Invalid Discovery Result Type!\r\n");
				break;
			}
		}
		break;

	case GCS_ALL_CHAR_DESC_DISCOV:
		APP_PRINT_INFO2("conn_id %d, GCS_ALL_CHAR_DESC_DISCOV, is_success %d\r\n",
						conn_id, discov_result.is_success);
		printf("conn_id %d, GCS_ALL_CHAR_DESC_DISCOV, is_success %d\r\n",
			   conn_id, discov_result.is_success);
		for (i = 0; i < discov_result.result_num; i++) {
			p_result_table = &(discov_result.p_result_table[i]);
			switch (p_result_table->result_type) {
			case DISC_RESULT_CHAR_DESC_UUID16:
				APP_PRINT_INFO3("DESC UUID16[%d]: Descriptors handle=0x%x, uuid16=<0x%x>",
								i, p_result_table->result_data.char_desc_uuid16_disc_data.handle,
								p_result_table->result_data.char_desc_uuid16_disc_data.uuid16);
				printf("DESC UUID16[%d]: Descriptors handle=0x%x, uuid16=<0x%x>\r\n",
					   i, p_result_table->result_data.char_desc_uuid16_disc_data.handle,
					   p_result_table->result_data.char_desc_uuid16_disc_data.uuid16);

				break;
			case DISC_RESULT_CHAR_DESC_UUID128:
				APP_PRINT_INFO3("DESC UUID128[%d]: Descriptors handle=0x%x, uuid128=<%b>",
								i, p_result_table->result_data.char_desc_uuid128_disc_data.handle,
								TRACE_BINARY(16, p_result_table->result_data.char_desc_uuid128_disc_data.uuid128));
				printf("DESC UUID128[%d]: Descriptors handle=0x%x, uuid128="UUID_128_FORMAT"\r\n",
					   i, p_result_table->result_data.char_desc_uuid128_disc_data.handle,
					   UUID_128(p_result_table->result_data.char_desc_uuid128_disc_data.uuid128));
				break;

			default:
				APP_PRINT_ERROR0("Invalid Discovery Result Type!");
				printf("Invalid Discovery Result Type!\r\n");
				break;
			}
		}
		break;

	default:
		APP_PRINT_ERROR2("Invalid disc type: conn_id %d, discov_type %d",
						 conn_id, discov_result.discov_type);
		printf("Invalid disc type: conn_id %d, discov_type %d\r\n",
			   conn_id, discov_result.discov_type);
		break;
	}
}
/**
 * @brief  Callback will be called when data sent from gcs client.
 * @param  client_id the ID distinguish which module sent the data.
 * @param  conn_id connection ID.
 * @param  p_data  pointer to data.
 * @retval   result @ref T_APP_RESULT
 */
T_APP_RESULT ble_scatternet_gcs_client_callback(T_CLIENT_ID client_id, uint8_t conn_id, void *p_data)
{
	T_APP_RESULT  result = APP_RESULT_SUCCESS;
	APP_PRINT_INFO2("ble_scatternet_gcs_client_callback: client_id %d, conn_id %d",
					client_id, conn_id);
	if (client_id == ble_scatternet_gcs_client_id) {
		T_GCS_CLIENT_CB_DATA *p_gcs_cb_data = (T_GCS_CLIENT_CB_DATA *)p_data;
		switch (p_gcs_cb_data->cb_type) {
		case GCS_CLIENT_CB_TYPE_DISC_RESULT:
			ble_scatternet_gcs_handle_discovery_result(conn_id, p_gcs_cb_data->cb_content.discov_result);
			break;
		case GCS_CLIENT_CB_TYPE_READ_RESULT:
			APP_PRINT_INFO3("READ RESULT: cause 0x%x, handle 0x%x, value_len %d",
							p_gcs_cb_data->cb_content.read_result.cause,
							p_gcs_cb_data->cb_content.read_result.handle,
							p_gcs_cb_data->cb_content.read_result.value_size);
			printf("READ RESULT: cause 0x%x, handle 0x%x, value_len %d\r\n",
				   p_gcs_cb_data->cb_content.read_result.cause,
				   p_gcs_cb_data->cb_content.read_result.handle,
				   p_gcs_cb_data->cb_content.read_result.value_size);

			if (p_gcs_cb_data->cb_content.read_result.cause == GAP_SUCCESS) {
				APP_PRINT_INFO1("READ VALUE: %b",
								TRACE_BINARY(p_gcs_cb_data->cb_content.read_result.value_size,
											 p_gcs_cb_data->cb_content.read_result.p_value));
				printf("READ VALUE: ");
				for (int i = 0; i < p_gcs_cb_data->cb_content.read_result.value_size; i++) {
					printf("0x%02x ", *(p_gcs_cb_data->cb_content.read_result.p_value + i));
				}
				printf("\r\n");
			}
			break;
		case GCS_CLIENT_CB_TYPE_WRITE_RESULT:
			APP_PRINT_INFO3("WRITE RESULT: cause 0x%x, handle 0x%x, type %d",
							p_gcs_cb_data->cb_content.write_result.cause,
							p_gcs_cb_data->cb_content.write_result.handle,
							p_gcs_cb_data->cb_content.write_result.type);
			printf("WRITE RESULT: cause 0x%x, handle 0x%x, type %d\r\n",
				   p_gcs_cb_data->cb_content.write_result.cause,
				   p_gcs_cb_data->cb_content.write_result.handle,
				   p_gcs_cb_data->cb_content.write_result.type);
			break;
		case GCS_CLIENT_CB_TYPE_NOTIF_IND:
			if (p_gcs_cb_data->cb_content.notif_ind.notify == false) {
				APP_PRINT_INFO2("INDICATION: handle 0x%x, value_size %d",
								p_gcs_cb_data->cb_content.notif_ind.handle,
								p_gcs_cb_data->cb_content.notif_ind.value_size);
				APP_PRINT_INFO1("INDICATION: value %b",
								TRACE_BINARY(p_gcs_cb_data->cb_content.notif_ind.value_size,
											 p_gcs_cb_data->cb_content.notif_ind.p_value));
				printf("INDICATION: handle 0x%x, value_size %d\r\n",
					   p_gcs_cb_data->cb_content.notif_ind.handle,
					   p_gcs_cb_data->cb_content.notif_ind.value_size);
				printf("INDICATION: value ");
				for (int i = 0; i < p_gcs_cb_data->cb_content.notif_ind.value_size; i++) {
					printf("0x%02x ", *(p_gcs_cb_data->cb_content.notif_ind.p_value + i));
				}
				printf("\r\n");
			} else {
				APP_PRINT_INFO2("NOTIFICATION: handle 0x%x, value_size %d",
								p_gcs_cb_data->cb_content.notif_ind.handle,
								p_gcs_cb_data->cb_content.notif_ind.value_size);
				APP_PRINT_INFO1("NOTIFICATION: value %b",
								TRACE_BINARY(p_gcs_cb_data->cb_content.notif_ind.value_size,
											 p_gcs_cb_data->cb_content.notif_ind.p_value));
				printf("NOTIFICATION: handle 0x%x, value_size %d\r\n",
					   p_gcs_cb_data->cb_content.notif_ind.handle,
					   p_gcs_cb_data->cb_content.notif_ind.value_size);
				printf("NOTIFICATION: value ");
				for (int i = 0; i < p_gcs_cb_data->cb_content.notif_ind.value_size; i++) {
					printf("0x%02x ", *(p_gcs_cb_data->cb_content.notif_ind.p_value + i));
				}
				printf("\r\n");
			}
			break;
		default:
			break;
		}
	}

	return result;
}

/** @} */ /* End of group GCS_CLIIENT_CALLBACK */
/** @} */ /* End of group CENTRAL_CLIENT_APP */
#endif
