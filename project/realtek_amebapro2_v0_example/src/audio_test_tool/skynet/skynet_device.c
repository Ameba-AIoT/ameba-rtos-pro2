#include "cmsis_os.h"
#include <time.h>
#include "device_lock.h"
#include "flash_api.h"
#include "wifi_conf.h"
#include "wifi_structures.h"
#include "lwip_netconf.h"
#include "SKYNET_IOTAPI.h"
#include "SKYNET_APP.h"
#include "device.h"
#include "serial_api.h"
#include "mmf2_module.h"
#include "avcodec.h"

#define FLASH_SYSTEM_DATA_ADDR			(0x1000000 - 0xA000)  // reserve 32K+4K for Image1 + Reserved data
#define P2P_SETTING_SECTOR		FLASH_SYSTEM_DATA_ADDR + 0x1000
#define P2P_SECTOR     P2P_SETTING_SECTOR
#define WIFI_SETTING_SECTOR		FLASH_SYSTEM_DATA_ADDR + 0x2000
#define WIFI_SECTOR     WIFI_SETTING_SECTOR
#define BACKUP_SECTOR	(FLASH_SYSTEM_DATA_ADDR - 0x1000)

#define P2P_UID_LEN				41//32 		///< Maximum of p2p uid length.

#define MAX_LISTEVENT_COUNT 50


/**
  * @brief  The structure is used to describe the setting when configure the p2p.
  */
typedef struct p2p_config {
	unsigned int		boot_mode;
	unsigned char 		uid[P2P_UID_LEN];
} p2p_config_t;


typedef struct p2p_wifi_config {
	unsigned int		boot_mode;
	unsigned char 		ssid[32];
	unsigned char		ssid_len;
	unsigned char		security_type;
	unsigned char		password[65];
	unsigned char		password_len;
	unsigned char		channel;
	unsigned char		user_pwd[65];
	unsigned char		record_mode;
	unsigned char		notice_sw;
	unsigned char		notice_mode;
	unsigned char		motion_level;
	unsigned char		cry_level;
} p2p_wifi_config_t;

/*P2P*/
int gProcessRun = 0;
extern int getIPNotice;
//extern int wait_for_i;
extern int rtsTimezone;
int sky_init_flag = 0;
static SemaphoreHandle_t  gSID_mutex;
SemaphoreHandle_t  cmd_q_mutex;
SemaphoreHandle_t  serial_q_mutex;
SemaphoreHandle_t  spk_q_mutex;
SemaphoreHandle_t  mic_q_mutex;
static QueueHandle_t xQueueMic = NULL;
static Queue *pQ;
static Queue *pQ_serial;
static Queue *pQ_spk;
static Queue *pQ_mic;
AV_Client gClientInfo[MAX_CLIENT_NUMBER];
SMsgAVIoctrlListEventResp *gEventList = NULL;
char gUID[32];
char gKey[8];
char gPass[16];
int loginFail = 0;
static int Record_Type = 0;
static int Light_SW = 0;
static int Light_Duty = 50;
static int Night_Light_SW = 0;
static int Night_Light_Duty = 1;
static int Motion_SW = 0;
static int Motion_Value = 0;
static int BattCap = 0;
static int skynetInitFlag = 0;
static int lSocketID = 0;
static int ghostSocketID = -1;
char gP2PString[41];
int UID_Setup = 0;
static int UID_Reset = 0;
int Config_Reset = 0;
int Wake_On_Wlan = 0;
char pb_file_path[64];
SMsgAVIoctrlPlayRecord pb_req;
int PB_CID;
int PB_SID;

/* WIFI AP */
extern struct netif xnetif[NET_IF_NUM];
extern int wifiReconnectTimes;
int startAPMode = 0;
int isAPMode = 0;
extern int wakeup_boot;
static int wifi_ap_sw = 0;
static int wifi_ap_on = 0;
static p2p_wifi_config_t wnet_config;
static int check_wifi_auth_flag = 0;
static rtw_security_t wifi_security_type = RTW_SECURITY_OPEN;
static unsigned int   wifi_channel = 1;
static unsigned char  wifi_ssid[32];
static int load_wifi_ap = 0;
static int start_sleep_cnt = 0;
char *ap_ssid;
char *ap_password;

/* fastconnect use wifi AT command. Not init_wifi_struct when log service disabled
 * static initialize all values for using fastconnect when log service disabled
 */
/*
static rtw_network_info_t wifi = {
	{},     // ssid
	{},     // bssid
	0,      // security
	0,      // password
	0,      // password len
	-1      // key id
};
*/

//static rtw_ap_info_t ap = {0};
//static unsigned char password[65] = {0};

st_GetWakeupInfo gstGetWakeupInfo;

static unsigned short ping_seq = 0;

#define US_PER_MS  (1000)

int update_wakeup_info = 0;
uint8_t wakeup_token[256], wakeup_ip[256], wakeup_uid[256];

#if 0
static int my_usleep(unsigned int usec)
{
	vTaskDelay((usec + US_PER_MS - 1) / US_PER_MS);
	return 0;
}

static void generate_ping_echo(unsigned char *buf, int size)
{
	int i;
	struct icmp_echo_hdr *pecho;

	for (i = 0; i < size; i ++) {
		buf[sizeof(struct icmp_echo_hdr) + i] = (unsigned char) i;
	}

	pecho = (struct icmp_echo_hdr *) buf;
	ICMPH_TYPE_SET(pecho, ICMP_ECHO);
	ICMPH_CODE_SET(pecho, 0);
	pecho->chksum = 0;
	pecho->id = 0xABCD;
	pecho->seqno = htons(++ ping_seq);

	//Checksum includes icmp header and data. Need to calculate after fill up icmp header
	pecho->chksum = inet_chksum(pecho, sizeof(struct icmp_echo_hdr) + size);
}

static void check_internet(void)
{
	struct hostent *server_host;
	struct sockaddr_in to_addr, from_addr;
	int from_addr_len = sizeof(struct sockaddr);
	int pint_timeout = 1000;
	int cnt = 0;
	int data_size = 32;
	unsigned char *ping_buf, *reply_buf;
	int ping_size, reply_size;
	int ping_socket;

	printf("\r\n************check_internet***********\r\n");

	ping_size = sizeof(struct icmp_echo_hdr) + data_size;

	ping_buf = pvPortMalloc(ping_size);
	if (NULL == ping_buf) {
		printf("\n\r[ERROR] %s: Allocate ping_buf failed", __func__);
		return;
	}

	reply_buf = pvPortMalloc(ping_size);
	if (NULL == reply_buf) {
		vPortFree(ping_buf);
		printf("\n\r[ERROR] %s: Allocate reply_buf failed", __func__);
		return;
	}

	ping_socket = socket(AF_INET, SOCK_RAW, IP_PROTO_ICMP);

	struct timeval timeout;
	timeout.tv_sec = pint_timeout / 1000;
	timeout.tv_usec = pint_timeout % 1000 * 1000;
	setsockopt(ping_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
	to_addr.sin_len = sizeof(to_addr);
	to_addr.sin_family = AF_INET;
	server_host = gethostbyname("8.8.8.8");
	if (inet_aton("8.8.8.8", &to_addr.sin_addr) == 0) {
		if (server_host == NULL) {
			printf("gethostbyname fail\r\n");
			vPortFree(ping_buf);
			vPortFree(reply_buf);
			return;
		} else {

		}
	} else {
		to_addr.sin_addr.s_addr = inet_addr("8.8.8.8");
	}

	generate_ping_echo(ping_buf, data_size);
	sendto(ping_socket, ping_buf, ping_size, 0, (struct sockaddr *) &to_addr, sizeof(to_addr));

	while (1) {

		reply_size = recvfrom(ping_socket, reply_buf, ping_size, 0, (struct sockaddr *) &from_addr, (socklen_t *) &from_addr_len);

		printf("@@@@@@@@@@ (%d) : %s\r\n", reply_size, reply_buf);
		/*
		if(reply_size > 0) {
			printf("Access Internet Success\r\n");
			break;
		} else {
			printf("Retry Access Internet\r\n");
		}
		*/

		cnt++;

		if (cnt > 10) {
			printf("check internet timeout\r\n");
			break;
		}
		vTaskDelay(200 / portTICK_PERIOD_MS);
	}

	vPortFree(ping_buf);
	vPortFree(reply_buf);
}
#endif

static void Save_WIFI_Config(void)
{
	flash_t flash;
	rtw_wifi_config_t wifi_config;
	uint32_t address;
	uint32_t data, i = 0;

	printf("Save_WIFI_Config\n");

	address = WIFI_SECTOR;

	flash_read_word(&flash, address, &data);
	if (data == ~0x0) {
		printf("save wifi config\n");
		device_mutex_lock(RT_DEV_LOCK_FLASH);
		flash_stream_write(&flash, address, sizeof(rtw_wifi_config_t), (unsigned char *)&wnet_config);
		device_mutex_unlock(RT_DEV_LOCK_FLASH);
		vTaskDelay(20);
		extern void sys_reset(void);
		sys_reset();
	} else {
		printf("save & erase wifi config\n");
		device_mutex_lock(RT_DEV_LOCK_FLASH);
		flash_erase_sector(&flash, BACKUP_SECTOR);
		for (i = 0; i < 0x1000; i += 4) {
			flash_read_word(&flash, WIFI_SECTOR + i, &data);
			if (i < sizeof(rtw_wifi_config_t)) {
				memcpy(&data, (char *)(&wnet_config) + i, 4);
				//printf("\n\r Wifi_config + %d = 0x%x",i,(void *)(&wnet_config + i));
				//printf("\n\r Data = %d",data);
			}
			flash_write_word(&flash, BACKUP_SECTOR + i, data);
		}
		flash_read_word(&flash, BACKUP_SECTOR + 68, &data);
		//printf("\n\r Base + BACKUP_SECTOR + 68 wifi channel = %d",data);
		//erase system data
		flash_erase_sector(&flash, WIFI_SECTOR);
		//write data back to system data
		for (i = 0; i < 0x1000; i += 4) {
			flash_read_word(&flash, BACKUP_SECTOR + i, &data);
			flash_write_word(&flash, WIFI_SECTOR + i, data);
		}
		//erase backup sector
		flash_erase_sector(&flash, BACKUP_SECTOR);
		device_mutex_unlock(RT_DEV_LOCK_FLASH);
		//vTaskDelay(20);
		//sys_reset();
	}
	//WIFI_Station_Mode(req.ssid,req.password);
	//sys_reset();
}

static void Load_WIFI_Config(void)
{
	flash_t flash;
	int ret = RTW_SUCCESS;

	p2p_wifi_config_t local_config;
	uint32_t address;

	address = WIFI_SECTOR;

	memset(&wnet_config, 0, sizeof(p2p_wifi_config_t));

	printf("\r\n==========  LoadWifiConfig(): Read from FLASH!==========\r\n");


	device_mutex_lock(RT_DEV_LOCK_FLASH);
	flash_stream_read(&flash, address, sizeof(p2p_wifi_config_t), (unsigned char *)(&local_config));
	device_mutex_unlock(RT_DEV_LOCK_FLASH);

	printf("\r\nLoadWifiConfig(): local_config.boot_mode=0x%x\n", local_config.boot_mode);
	printf("\r\nLoadWifiConfig(): local_config.ssid(%d)=%s\n", strlen((const char *)local_config.ssid), local_config.ssid);
	printf("\r\nLoadWifiConfig(): local_config.channel=%d\n", local_config.channel);
	printf("\r\nLoadWifiConfig(): local_config.security_type=%d\n", local_config.security_type);
	printf("\r\nLoadWifiConfig(): local_config.password=%s\n", local_config.password);
	printf("\r\nLoadWifiConfig(): local_config.user_pwd=%s\n", local_config.user_pwd);
	printf("\r\nLoadWifiConfig(): local_config.record_mode=%d\n", local_config.record_mode);
	printf("\r\nLoadWifiConfig(): local_config.notice_sw=%d\n", local_config.notice_sw);
	printf("\r\nLoadWifiConfig(): local_config.notice_mode=%d\n", local_config.notice_mode);
	printf("\r\nLoadWifiConfig(): local_config.motion_level=%d\n", local_config.motion_level);
	printf("\r\nLoadWifiConfig(): local_config.cry_level=%d\n", local_config.cry_level);

	if (local_config.boot_mode == 0x77665502) {
		printf("\r\nLoad_WIFI_Config(): Read from FLASH Success!\r\n");
		memcpy(&wnet_config, &local_config, sizeof(p2p_wifi_config_t));
	} else {
		printf("\r\nLoad_WIFI_Config(): Read from FLASH Fail!\r\n");
		memset(&wnet_config, 0x00, sizeof(p2p_wifi_config_t));
		sprintf((char *)wnet_config.user_pwd, "12345678");
		Save_WIFI_Config();
	}

#if 0
	if ((local_config.ssid_len > 0) && (local_config.password_len > 0)) {

		if (local_config.boot_mode == 0x77665502) {
			printf("\r\n========= Boot Station Mode =========\r\n");
			local_config.boot_mode = 0x77665500;
			//wifi_set_autoreconnect(0);
			wifi_on(RTW_MODE_STA);
			device_mutex_lock(RT_DEV_LOCK_FLASH);
			flash_stream_write(&flash, address, sizeof(p2p_wifi_config_t), (unsigned char *)&local_config);
			device_mutex_unlock(RT_DEV_LOCK_FLASH);
			check_wifi_auth_flag = 0;
			memset(wifi_ssid, 0, 32);
			strcpy(wifi_ssid, local_config.ssid);
			printf("1.Check Security Type\r\n");
			vTaskDelay(2000);
			if ((ret = wifi_scan_networks(app_scan_result_handler, NULL)) != RTW_SUCCESS) {
				printf("ERROR: wifi scan failed\n\r");

			}
			printf("2.Check Security Type : %d\r\n", ret);
			vTaskDelay(3000);
			printf("Check Security Type Done : %d / %d\r\n", check_wifi_auth_flag, wifi_security_type);

			RTW_API_INFO(("Security : %s\r\n", (wifi_security_type == RTW_SECURITY_OPEN) ? "Open" :
						  (wifi_security_type == RTW_SECURITY_WEP_PSK) ? "WEP" :
						  (wifi_security_type == RTW_SECURITY_WPA_TKIP_PSK) ? "WPA TKIP" :
						  (wifi_security_type == RTW_SECURITY_WPA_AES_PSK) ? "WPA AES" :
						  (wifi_security_type == RTW_SECURITY_WPA2_AES_PSK) ? "WPA2 AES" :
						  (wifi_security_type == RTW_SECURITY_WPA2_TKIP_PSK) ? "WPA2 TKIP" :
						  (wifi_security_type == RTW_SECURITY_WPA2_MIXED_PSK) ? "WPA2 Mixed" :
						  (wifi_security_type == RTW_SECURITY_WPA_WPA2_MIXED) ? "WPA/WPA2 AES" :
						  "Unknown"));


			if (check_wifi_auth_flag == 1) {
				printf("check_wifi_auth_flag == 1\r\n");
				WIFI_Station_Mode(local_config.ssid, local_config.password, wifi_security_type);
			} else {
				printf("check_wifi_auth_flag == 0\r\n");
				WIFI_Station_Mode(local_config.ssid, local_config.password, RTW_SECURITY_WPA2_AES_PSK);
			}
			load_wifi_ap = 0;
			wifiReconnectTimes = 0;
			wifi_set_autoreconnect(1);
			//vTaskDelay(200);
			//sys_reset();
		} else if (local_config.boot_mode == 0x77665503) {
			printf("\r\n========== Boot AP Mode ===========\r\n");
			//wifi_set_autoreconnect(0);
			WIFI_AP_Mode();
			load_wifi_ap = 1;
			//Restore_Station_WIFI_Config();
		} else if (local_config.boot_mode == 0x77665504) {
			printf("\r\n========== Boot Tmp AP Mode ===========\r\n");
			local_config.boot_mode = 0x77665500;
			//wifi_set_autoreconnect(0);
			device_mutex_lock(RT_DEV_LOCK_FLASH);
			flash_stream_write(&flash, address, sizeof(p2p_wifi_config_t), (unsigned char *)&local_config);
			device_mutex_unlock(RT_DEV_LOCK_FLASH);
			wifi_set_autoreconnect(0);
			WIFI_AP_Mode();
			load_wifi_ap = 1;
			//Restore_Station_WIFI_Config();
		} else if (local_config.boot_mode == 0xffffffff) {
			printf("\r\n========== Boot AP Mode 0xffffffff===========\r\n");
			//wifi_set_autoreconnect(0);
			WIFI_AP_Mode();
			load_wifi_ap = 1;
			//Restore_Station_WIFI_Config();
		} else if (local_config.boot_mode == 0) {
			printf("\r\n========== Boot AP Mode 0x0===========\r\n");
			//wifi_set_autoreconnect(0);
			WIFI_AP_Mode();
			load_wifi_ap = 1;
			//Restore_Station_WIFI_Config();
		} else {
			printf("\r\n========= Boot Keep Station Mode =========\r\n");
			wifi_on(RTW_MODE_STA);
			wifi_set_autoreconnect(1);
		}
	} else {
		printf("\r\n========== Boot AP Mode None SSID & PASSWD ========\r\n");
		//wifi_set_autoreconnect(0);
		WIFI_AP_Mode();
		load_wifi_ap = 1;
	}
#endif
}

unsigned long myGetTickCount(void)
{
	unsigned long currentTime = 0;
	struct timeval current;
	//gettimeofday(&current, NULL);
	//currentTime = current.tv_sec * 1000 + current.tv_usec / 1000;
	return currentTime;
}

int isEmpty(Queue *pQueue)
{
	if (pQueue == NULL) {
		return FALSE;
	}
	if (pQueue->size == 0) {
		return TRUE;
	} else {
		return FALSE;
	}
}

int Enqueue(Queue *pQueue, NODE *item)
{
	/* Bad parameter */
	if ((pQueue == NULL) || (item == NULL)) {
		return FALSE;
	}
	// if(pQueue->limit != 0)
	if (pQueue->size >= pQueue->limit) {
		return FALSE;
	}
	/*the queue is empty*/
	item->prev = NULL;
	if (pQueue->size == 0) {
		pQueue->head = item;
		pQueue->tail = item;

	} else {
		/*adding item to the end of the queue*/
		pQueue->tail->prev = item;
		pQueue->tail = item;
	}
	pQueue->size++;
	return TRUE;
}

NODE *Dequeue(Queue *pQueue)
{
	/*the queue is empty or bad param*/
	NODE *item;
	if (isEmpty(pQueue)) {
		return NULL;
	}
	item = pQueue->head;
	pQueue->head = (pQueue->head)->prev;
	pQueue->size--;
	return item;
}

Queue *ConstructQueue(int limit)
{
	Queue *queue = (Queue *) malloc(sizeof(Queue));
	if (queue == NULL) {
		return NULL;
	}
	if (limit <= 0) {
		limit = 65535;
	}
	queue->limit = limit;
	queue->size = 0;
	queue->head = NULL;
	queue->tail = NULL;

	return queue;
}

void DestructQueue(Queue *queue)
{
	NODE *pN;
	while (!isEmpty(queue)) {
		pN = Dequeue(queue);
		free(pN);
	}
	free(queue);
}

void regedit_client_to_video(int CID)
{
	AV_Client *p = &(gClientInfo[CID]);
	p->bEnableVideo = 1;
	printf("regedit_client_to_video\n\r");
}

void unregedit_client_from_video(int CID)
{
	AV_Client *p = &(gClientInfo[CID]);
	p->bEnableVideo = 0;
	printf("unregedit_client_from_video\n\r");
	//PPCS_Close(gClientInfo[SID][ch].SID);
}

void regedit_client_to_audio(int CID)
{
	AV_Client *p = &(gClientInfo[CID]);
	p->bEnableAudio = 1;

}

void unregedit_client_from_audio(int CID)
{
	AV_Client *p = &(gClientInfo[CID]);
	p->bEnableAudio = 0;
}

static int myGetDataSizeFrom(st_AVStreamIOHead *pStreamIOHead)
{
	int nDataSize = pStreamIOHead->nStreamIOHead;
	nDataSize &= 0x00FFFFFF;
	return nDataSize;
}

void putSerialPacket(char *nSrcBuf, int nDataSize)
{
	NODE *pN;
	pN = (NODE *) malloc(sizeof(NODE));
	pN->data.buffer = (char *)malloc(sizeof(char) * nDataSize);
	if (pN->data.buffer == NULL) {
		printf("putCMDPacket malloc Fail %d\n", nDataSize);
	}
	pN->data.buffer_length = nDataSize;
	memcpy(pN->data.buffer, nSrcBuf, nDataSize);
	xSemaphoreTake(serial_q_mutex, portMAX_DELAY);
	Enqueue(pQ_serial, pN);
	xSemaphoreGive(serial_q_mutex);

	//printf("kenneth putCMDPacket Finish\n");
}

int getSerialPacket(char *nDestBuf)
{
	int nDataSize = 0;
	NODE *pN;

	if (!isEmpty(pQ_serial)) {
		xSemaphoreTake(serial_q_mutex, portMAX_DELAY);
		pN = Dequeue(pQ_serial);
		memcpy(nDestBuf, pN->data.buffer, pN->data.buffer_length);
		nDataSize = pN->data.buffer_length ;
		free(pN->data.buffer);
		free(pN);
		xSemaphoreGive(serial_q_mutex);
		//printf("kenneth getCMDPacket Finish\n");
	}

	return nDataSize;
}

void putCMDPacket(int nSocketID, int nCID, char *nSrcBuf, int nDataSize)
{
	NODE *pN;
	pN = (NODE *) malloc(sizeof(NODE));
	pN->data.buffer = (char *)malloc(sizeof(char) * nDataSize);
	if (pN->data.buffer == NULL) {
		printf("putCMDPacket malloc Fail %d\n", nDataSize);
	}
	pN->data.nSocketID = nSocketID;
	pN->data.nCID = nCID;
	pN->data.buffer_length = nDataSize;
	memcpy(pN->data.buffer, nSrcBuf, nDataSize);
	xSemaphoreTake(cmd_q_mutex, portMAX_DELAY);
	Enqueue(pQ, pN);
	xSemaphoreGive(cmd_q_mutex);

	//printf("kenneth putCMDPacket Finish\n");
}

int getCMDPacket(int *SID, int *CID, char *nDestBuf)
{
	int nDataSize = 0;
	NODE *pN;

	if (!isEmpty(pQ)) {
		xSemaphoreTake(cmd_q_mutex, portMAX_DELAY);
		pN = Dequeue(pQ);
		memcpy(nDestBuf, pN->data.buffer, pN->data.buffer_length);
		nDataSize = pN->data.buffer_length ;
		*SID = pN->data.nSocketID;
		*CID = pN->data.nCID;
		free(pN->data.buffer);
		free(pN);
		xSemaphoreGive(cmd_q_mutex);
		//printf("kenneth getCMDPacket Finish\n");
	}

	return nDataSize;
}

static void Save_P2P_Config(void)
{
	flash_t flash;
	p2p_config_t p_config;
	uint32_t address;
	uint32_t data, i = 0;

	// clean wifi_config first
	memset(&p_config, 0x00, sizeof(p2p_config_t));

	address = P2P_SECTOR;

	p_config.boot_mode = 0x77665503;

	printf("gP2PString:%s\r\n", gP2PString);

	strncpy((char *)p_config.uid, (char *)gP2PString, 40);
#if 0
	//flash_read_word(&flash,address,&data);

	if (data == ~0x0) {
		printf("save p2p config\r\n");
		device_mutex_lock(RT_DEV_LOCK_FLASH);
		flash_stream_write(&flash, P2P_SECTOR, sizeof(p2p_config_t), (unsigned char *)&p_config);
		device_mutex_unlock(RT_DEV_LOCK_FLASH);
		vTaskDelay(20);
		//sys_reset();
	} else {
#else
	{
#endif
		printf("save & erase p2p config\r\n");
		device_mutex_lock(RT_DEV_LOCK_FLASH);
		flash_erase_sector(&flash, BACKUP_SECTOR);
		for (i = 0; i < 0x1000; i += 4)
		{
			flash_read_word(&flash, P2P_SECTOR + i, &data);
			if (i < sizeof(p2p_config_t)) {
				memcpy(&data, (char *)(&p_config) + i, 4);
			}
			flash_write_word(&flash, BACKUP_SECTOR + i, data);
		}
		//erase system data
		flash_erase_sector(&flash, P2P_SECTOR);
		//write data back to system data
		for (i = 0; i < 0x1000; i += 4)
		{
			flash_read_word(&flash, BACKUP_SECTOR + i, &data);
			flash_write_word(&flash, P2P_SECTOR + i, data);
		}
		//erase backup sector
		flash_erase_sector(&flash, BACKUP_SECTOR);
		device_mutex_unlock(RT_DEV_LOCK_FLASH);
		vTaskDelay(20);
	}
}


static void Load_P2P_Config(void)
{
	flash_t flash;
	const char *delim = ",";
	char *pch;
	int p_cnt = 0;
	p2p_config_t local_config;
	uint32_t address;

	memset(gUID, 0, 32);
	memset(gKey, 0, 8);

	address = P2P_SECTOR;

	printf("\r\nLoadP2PConfig(%d): Read from FLASH!\r\n", sizeof(p2p_config_t));

	device_mutex_lock(RT_DEV_LOCK_FLASH);
	flash_stream_read(&flash, P2P_SECTOR/*address*/, sizeof(p2p_config_t), (uint8_t *)(&local_config));
	device_mutex_unlock(RT_DEV_LOCK_FLASH);

	printf("\r\nLoadP2PConfig(): local_config.boot_mode=0x%x\r\n", local_config.boot_mode);
	printf("\r\nLoadP2PConfig(): local_config.uid=%s\r\n", local_config.uid);

	if (local_config.boot_mode == 0x77665503) {
		printf("\r\nLoadP2PConfig(): Read from FLASH Success!\r\n");
		pch = strtok((char *)local_config.uid, delim);
		while (pch != NULL) {
			if (p_cnt == 0) {
				strncpy(gUID, pch, strlen(pch));
				gUID[strlen(pch) + 1] = '\0';
			} else {
				strncpy(gKey, pch, strlen(pch));
				gKey[strlen(pch) + 1] = '\0';
			}
			pch = strtok(NULL, delim);
			p_cnt++;
		}
		printf("UID:%s\r\n", gUID);
		printf("KEY:%s\r\n", gKey);
	} else {
		printf("\r\nLoadP2PConfig(): Read from FLASH Fail!\r\n");
		printf("Setup Default UID\r\n");
		strcpy(gUID, "PIXMAX-00000088-99D81");
		strcpy(gKey, "437750");
		strcpy(gPass, "12345678");
	}

}

void initAVInfo(void)
{
	int i, j;
	for (i = 0; i < MAX_CLIENT_NUMBER; i++) {
		memset(&(gClientInfo[i]), 0, sizeof(AV_Client));
		gClientInfo[i].SID = -1;
		gClientInfo[i].destroyFlag = 0;
		printf("Intial gClientInfo[i].pBuf_mutex\r\n");
		gClientInfo[i].pBuf_mutex = xSemaphoreCreateMutex();
	}

	printf("Intial gClientInfo[i].pBuf_mutex\r\n");
	gSID_mutex = xSemaphoreCreateMutex();
	cmd_q_mutex = xSemaphoreCreateMutex();
	serial_q_mutex = xSemaphoreCreateMutex();


	pQ = ConstructQueue(7);
	pQ_serial = ConstructQueue(7);
	gEventList = (SMsgAVIoctrlListEventResp *)malloc(sizeof(SMsgAVIoctrlListEventResp) + sizeof(SAvEvent) * MAX_LISTEVENT_COUNT);
	memset(gEventList, 0, sizeof(SMsgAVIoctrlListEventResp));

	memset(gUID, 0, 25);
	memset(gKey, 0, 8);
	memset(gPass, 0, 16);

	Load_P2P_Config();

	sprintf((char *)wnet_config.user_pwd, "12345678");
}

void resetClients(void)
{
	int i;
	printf("\n##############resetClients###############\r\n");


	for (i = 0; i < MAX_CLIENT_NUMBER; i++) {
		gClientInfo[i].bEnableVideo = 0;
		gClientInfo[i].bEnableAudio = 0;
		gClientInfo[i].failCnt = 0;
		//gClientInfo[i].destroyFlag = 0;
		//if(gClientInfo[i].recvBuf != NULL)
		//	free(gClientInfo[i].recvBuf);
		//memset(&(gClientInfo[i]), 0, sizeof(AV_Client));

	}

	//vTaskDelay(200);

	for (i = 0; i < MAX_CLIENT_NUMBER; i++) {
		if (gClientInfo[i].SID > 0) {
			printf("792*********** %d\n", i);
			SKYNET_close(gClientInfo[i].SID, HOW_SHUT_RDWR);
			gClientInfo[i].SID = -1;
		}

	}
}

static int findFreeClient(void)
{
	int i;
	xSemaphoreTake(gSID_mutex, portMAX_DELAY);
	for (i = 0; i < MAX_CLIENT_NUMBER; i++) {
		printf("1.findFreeClient %d=%d\r\n", i, gClientInfo[i].SID);
		if (gClientInfo[i].SID == -1) {
			printf("\r\n++++++ findFreeClient(%d) ++++++\r\n", i);
			xSemaphoreGive(gSID_mutex);
			return i;
		}
	}
	printf("\r\n++++++ findFreeClient Fail ++++++\r\n");
	xSemaphoreGive(gSID_mutex);
	return -1;
}

static int checkClientConnectionCount(void)
{
	int i;
	int cnt = 0;
	xSemaphoreTake(gSID_mutex, portMAX_DELAY);
	for (i = 0; i < MAX_CLIENT_NUMBER; i++) {
		if (gClientInfo[i].SID > 0) {
			cnt++;
		}
	}
	xSemaphoreGive(gSID_mutex);
	return cnt;
}

static int findClient(int SID)
{
	int i;
	xSemaphoreTake(gSID_mutex, portMAX_DELAY);
	for (i = 0; i < MAX_CLIENT_NUMBER; i++) {
		if (gClientInfo[i].SID == SID) {
			//fprintf(stderr,"\n++++++ findClient(%d) ++++++\n",i);
			xSemaphoreGive(gSID_mutex);
			return i;
		}
	}
	//fprintf(stderr,"\n++++++ findClient Fail ++++++\n");
	xSemaphoreGive(gSID_mutex);
	return -1;
}

static int checkDestoryClient(void)
{
	int i;
	//printf("1.checkDestoryClient\r\n");
	xSemaphoreTake(gSID_mutex, portMAX_DELAY);
	//printf("2.checkDestoryClient\r\n");
	for (i = 0; i < MAX_CLIENT_NUMBER; i++) {
		//printf("3.checkDestoryClient %d\r\n",i);
		//if((gClientInfo[i].destroyFlag == 1) && (gClientInfo[i].bEnableVideo == 0)) {
		if (gClientInfo[i].destroyFlag == 1) {
			//printf("++++++ checkDestoryClient(%d) ++++++\r\n",i);
			xSemaphoreGive(gSID_mutex);
			return i;
		}
		//printf("4.checkDestoryClient\r\n");
	}
	//printf("5.checkDestoryClient\r\n");
	xSemaphoreGive(gSID_mutex);
	//printf("6.checkDestoryClient\r\n");
	return -1;
}

static void freeClient(int CID)
{
	printf("\r\n===== freeClient:%d/%d =====\r\n", CID, gClientInfo[CID].SID);

	//xSemaphoreTake(gSID_mutex, portMAX_DELAY);
	gClientInfo[CID].destroyFlag = 0;
	//printf("1.freeClient\r\n");
	gClientInfo[CID].bEnableVideo = 0;
	//printf("2.freeClient\r\n");
	gClientInfo[CID].bEnableAudio = 0;
	//printf("3.freeClient\r\n");
	gClientInfo[CID].failCnt = 0;
	//printf("4.freeClient\r\n");
	gClientInfo[CID].login = 0;
	//printf("5.freeClient\r\n");
	//xSemaphoreTake(gSID_mutex, portMAX_DELAY);
	//printf("6.freeClient\r\n");
	//printf("10.freeClient\r\n");
	//xSemaphoreGive(gSID_mutex);
	//printf("11.freeClient\r\n");
	SKYNET_close(gClientInfo[CID].SID, HOW_SHUT_RDWR);
	//printf("12.freeClient\r\n");
	gClientInfo[CID].SID = -1;
	//xSemaphoreGive(gSID_mutex);
	if (gClientInfo[CID].recvBuf != NULL) {
		//printf("7.freeClient\r\n");
		free(gClientInfo[CID].recvBuf);
		//printf("8.freeClient\r\n");
		gClientInfo[CID].recvBuf = NULL;
		//printf("9.freeClient\r\n");
	}
	printf("\r\n<<<<<<<< %d / %d >>>>>>>>>>>\r\n", CID, gClientInfo[CID].SID);
	//printf("13.freeClient\r\n");
}

int skyNetModuleDeinit(void)
{
	int nRet = SKYNET_Module_Deinit(1 << 1);
	printf("SKYNET_Module_Deinit ret = %d\r\n", nRet);
	sky_init_flag = 0;

	return nRet;
}

int skyNetModuleListen(void)
{
	printf("33333333333333333333\r\n");
	lSocketID = SKYNET_socket(0);

	printf("\r\n=========== [thread_listen] open socket ID = %d =============\r\n", lSocketID);

	if (lSocketID < 0) {
		printf("########## Open Socket Fail############\r\n");
		return -1;
	}

	st_DeviceInfo stDeviceInfo;
	stDeviceInfo.eDeviceType = DEVICETYPE_UNKNOWN;
	stDeviceInfo.eIcType = ICTYPE_REALTEK;
	stDeviceInfo.eOsType = OSTYPE_FREERTOS;
	stDeviceInfo.nOsVersion = FREERTOS_VERSION;
	stDeviceInfo.isEnableWakeupFunction = 1;

	//int nRet = SKYNET_listen(nSocketID, "PIXMAX-00000055-D9923", "FB2407", "SKYNET Ameba", MAX_SESSION_HANDLER_NUM, stDeviceInfo);
	//int nRet = SKYNET_listen(lSocketID, "PIXMAX-00000019-6F144", "90710B", "SKYNET Ameba", MAX_SESSION_HANDLER_NUM, &stDeviceInfo);
	int nRet = SKYNET_listen(lSocketID, gUID, gKey, "SKYNET Ameba", MAX_SESSION_HANDLER_NUM, &stDeviceInfo);

	printf("SKYNET_listen ret = %d , UID:%s Key:%s\r\n", nRet, gUID, gKey);

	return nRet;
}

extern int audio_handle(void *p, void *input, void *output);

char audio_data[1280];
mm_queue_item_t audio_output_item;

int SocketRecvCB(int nSocketID, int nSize, void *pUserData)
{
	int CID = 0;
	int nRet = 0;
	int totalSize = 0;

	CID = findClient(nSocketID);

	//printf("SocketRecvCB nSocketID:%d nSize:%d CID:%d\r\n",nSocketID,nSize,CID);

	//CID = findClient(nSocketID);

	if (nSize > 0) {
		if (gClientInfo[CID].recvRemainLen == 0) {
			gClientInfo[CID].recvRemainLen = nSize;
			if (gClientInfo[CID].recvTotalRemainLen <= 0) {
				gClientInfo[CID].recvPos = 0;
			}
			if (gClientInfo[CID].recvBuf != NULL) {
				nRet = SKYNET_recv(nSocketID, gClientInfo[CID].recvBuf + gClientInfo[CID].recvPos, nSize);
			}
		} else {
			if (gClientInfo[CID].recvBuf != NULL) {
				nRet = SKYNET_recv(nSocketID, gClientInfo[CID].recvBuf + gClientInfo[CID].recvPos, gClientInfo[CID].recvRemainLen);
			}
		}

		if (nRet < 0) {
			printf("IOT_RECV_ERROR_OCCURRED\r\n");
			gClientInfo[CID].destroyFlag = 1;
			return IOT_RECV_ERROR_OCCURRED;
		} else if (nRet == 0) {
			return IOT_RECV_NO_DATA_CAN_READ;
		} else {
			gClientInfo[CID].recvRemainLen -= nRet;
			gClientInfo[CID].recvPos += nRet;
			if (gClientInfo[CID].recvRemainLen < 0) {
				gClientInfo[CID].recvRemainLen = 0;
			}
		}

		if (gClientInfo[CID].recvRemainLen == 0) {
			st_AVStreamIOHead *pStreamIOHead = (st_AVStreamIOHead *)(gClientInfo[CID].recvBuf);

			if (gClientInfo[CID].recvTotalRemainLen <= 0) {
				totalSize = sizeof(st_AVStreamIOHead) + myGetDataSizeFrom(pStreamIOHead);
				gClientInfo[CID].recvTotalRemainLen = totalSize - gClientInfo[CID].recvPos;
			}

			if (gClientInfo[CID].recvTotalRemainLen <= 0) {
				if (pStreamIOHead->uionStreamIOHead.nStreamIOType == SIO_TYPE_IOCTRL) {
					putCMDPacket(nSocketID, CID, gClientInfo[CID].recvBuf + sizeof(st_AVStreamIOHead), gClientInfo[CID].recvPos);
				} else if (pStreamIOHead->uionStreamIOHead.nStreamIOType == SIO_TYPE_AUDIO) {
					//printf("SocketRecvCB SIO_TYPE_AUDIO %d/%d\n",gClientInfo[CID].recvBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVFrameHead), gClientInfo[CID].recvPos - sizeof(st_AVStreamIOHead) - sizeof(st_AVFrameHead));
					//putSPKPacket(nSocketID ,CID, gClientInfo[CID].recvBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVFrameHead), gClientInfo[CID].recvPos - sizeof(st_AVStreamIOHead) - sizeof(st_AVFrameHead));
					//mm_context_t *mctx = (mm_context_t*)g711d_ctx;

					int i = 0;
					for (i = 0; i < 2; i++) {
						while (audio_output_item.flag == 1) {
							vTaskDelay(1);
						}
						audio_output_item.data_addr = (uint32_t)audio_data;
						audio_output_item.size = 320;//gClientInfo[CID].recvPos - sizeof(st_AVStreamIOHead) - sizeof(st_AVFrameHead);
						memcpy((void *)audio_output_item.data_addr, (void *)(gClientInfo[CID].recvBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVFrameHead) + i * 320), 320);
						audio_output_item.type = AV_CODEC_ID_PCM_RAW;
						audio_output_item.timestamp = xTaskGetTickCount();
						audio_output_item.in_idx = 0;
						audio_output_item.flag = 1;
						//printf("size =%d\r\n",audio_output_item.size);
					}

				} else if (pStreamIOHead->uionStreamIOHead.nStreamIOType == SIO_TYPE_VIDEO) {
					printf("SocketRecvCB SIO_TYPE_VIDEO\r\n");
				} else {
					printf("SocketRecvCB SIO_TYPE_UNKN\r\n");
				}
			}

			return IOT_RECV_ALREADY_GOT_COMPLETE_DATA ;
		} else if (gClientInfo[CID].recvRemainLen < 0) {
			printf("SocketRecvCB Unknow Status\r\n");
		}

	} else {
		if (CID < 0) {
			printf("\r\n<<<<< Ghost SocketRecvCB Error nSocketID:%d CID: %d nSize: %d >>>>>>\r\n", nSocketID, CID, nSize);
			ghostSocketID = nSocketID;
		} else {
			printf("\r\n====== SocketRecvCB Error nSocketID:%d CID: %d nSize: %d ======\r\n", nSocketID, CID, nSize);
			gClientInfo[CID].destroyFlag = 1;
		}
		return IOT_RECV_ERROR_OCCURRED;
	}
	return IOT_RECV_WANT_CONTINUE_TO_READ;
}


void DeviceLoginCB(int nSocketID, int nResult, void *pUserData)
{
	printf("\r\n<<<< [LoginCB] invoke nSocketID = %d, nResult = %d >>>>\r\n", nSocketID, nResult);

	//printf("[LoginCB] invoke nSocketID = %d, nResult = %d, pUserData = %s\n", nSocketID, nResult, pUserData);
	if (nResult < 0) {
		if ((nResult != SKYNET_ERR_ID_ILLEGAL) &&
			(nResult != SKYNET_ERR_ICESERVER_EMPTY) &&
			(nResult != SKYNET_ERR_ID_DUPLICATE) &&
			(nResult != SKYNET_ERR_ID_EXPIRED) &&
			(nResult != SKYNET_ERR_ID_BLACKLIST)) {
			printf("Login Fail\n");
			if (isAPMode == 0) {
				loginFail = 1;
			}
			//getIPNotice = 1;
		} else {
			printf("ID problem\r\n");
		}

	} else {
		printf("Login success\r\n");
		loginFail = 0;


		int ret = SKYNET_getsockopt(nSocketID, SOCKOPT_GET_WAKEUP_INFO, (void *)&gstGetWakeupInfo);

		if (ret == 0) {
			printf("[WAKEUP INFO]\r\n");
			printf("\tWakeup server number = %d\r\n", gstGetWakeupInfo.nServerNumber);
			if (gstGetWakeupInfo.nServerNumber > 0) {
				int i;
				for (i = 0; i < gstGetWakeupInfo.nServerNumber; i++) {
					memset(wakeup_token, 0, 256);
					memset(wakeup_ip, 0, 256);
					memset(wakeup_uid, 0, 256);

					sprintf((char *)wakeup_token, "IPCtoken=%.40s", gstGetWakeupInfo.WakeupToken[i]);
					sprintf((char *)wakeup_ip, "IPCserver_ip=%s", gstGetWakeupInfo.ServerIP[i]);
					sprintf((char *)wakeup_uid, "IPCuid=%s", gUID);
					printf("\tWakeup token = %.40s\r\n", gstGetWakeupInfo.WakeupToken[i]);
					printf("\tWakeup server ip = %s\r\n", gstGetWakeupInfo.ServerIP[i]);
					update_wakeup_info = 1;
				}
				start_sleep_cnt = 1;
			} else {
				printf("No wakeup server!!!\r\n");
			}
		} else {
			printf("[Error] SKYNET_getsockopt with SOCKOPT_GET_WAKEUP_INFO ret = %d\r\n", ret);
		}
	}
}

void SocketConnectCB(const char *pszID, int nSocketID, int nResult, void *pUserData)
{
	int CID = findFreeClient();
	int nRet;

	printf("[SocketConnectCB] invoke pszID = %s, nSocketID = %d, nResult = %d\r\n", pszID, nSocketID, nResult);

	if ((CID >= 0) && (nResult >= 0)) {
		nRet = SKYNET_alloc_buf(nSocketID, &gClientInfo[CID].pBuf, MAX_SEND_BUF_SIZE);
		if (nRet < 0) {
			printf("!!!SKYNET_alloc_buf() failed nRet = %d!!!\r\n", nRet);
		}
		gClientInfo[CID].destroyFlag = 0;
		gClientInfo[CID].failCnt = 0;
		gClientInfo[CID].login = 0;
		gClientInfo[CID].bEnableVideo = 0;
		gClientInfo[CID].bEnableAudio = 0;
		gClientInfo[CID].SID = nSocketID;
		gClientInfo[CID].recvBuf = (char *) malloc(MAX_SEND_BUF_SIZE);
		gClientInfo[CID].recvRemainLen = 0;
		gClientInfo[CID].recvPos = 0;
		gClientInfo[CID].recvRemainLen = 0;
		if (gClientInfo[CID].recvBuf == NULL) {
			printf("recvBuf alloc Fail\n");
		}


		st_ConnectionInfo stConnectionInfo;
		nRet = SKYNET_getsockopt(nSocketID, SOCKOPT_CONNECTION_INFO, (void *)&stConnectionInfo);
		if (nRet == 0) {
			printf("[Connection information]\r\n");
			printf("\tMode = %d, RemoteVersion = %X\r\n", stConnectionInfo.ConnMode, stConnectionInfo.RemoteVersion);
			printf("\tLocalNatType = %d, RemoteNatType = %X\r\n", stConnectionInfo.LocalNatType, stConnectionInfo.RemoteNatType);
			printf("\tMy Wan IP = %s:%d\r\n", stConnectionInfo.WanIP, stConnectionInfo.WanPort);
			printf("\tMy Local IP = %s:%d\r\n", stConnectionInfo.LocalIP, stConnectionInfo.LocalPort);
			printf("\tRemote IP = %s:%d\n", stConnectionInfo.RemoteIP, stConnectionInfo.RemotePort);
			printf("\tGotData = %d, Role = %d\r\n", stConnectionInfo.GotData, stConnectionInfo.Role);

		} else {
			printf("@@ SKYNET_getsockopt() error = %d\n", nRet);
		}
	} else {
		printf("SocketConnectCB Fail ErrCode:%d\r\n", nResult);
		if (CID >= 0) {
			gClientInfo[CID].destroyFlag = 1;
		}
	}
}

int skyNetModuleInit(void)
{
	printf("11111111111111111111111\r\n");
	if (sky_init_flag == 1) {
		printf("skyNetModuleInit Fail\r\n");
		return -1;
	}
	printf("222222222222222222222222\r\n");
	int nRet = SKYNET_Module_Init(1 << 1);

	printf("\r\n======== SKYNET_Module_Init ret = %d ==========\r\n", nRet);
	SKYNET_NotReuseLocalPort();

	printf("Module Version = %s\r\n", SKYNET_Module_Version(1 << 1));


	SKYNET_set_UUID("0123456789012345678901234567890123456789012345678901234567891234");

	SKYNET_set_recv_callback(SocketRecvCB, NULL);

	SKYNET_set_connect_callback(SocketConnectCB, NULL);

	SKYNET_set_login_callback(DeviceLoginCB, NULL);

	sky_init_flag = 1;

	return nRet;
}

int speaker_start = 0;
void Handle_IOCTRL_Cmd(int SID, int CID, char *buf)
{
	int nRet, i, j;
	char sendBuf[512];
	st_AVIOCtrlHead stIOCtrlHead;

	//printf("!!!! Handle_IOCTRL_Cmd %x %x %x %x %x %x %x %x\n",buf[0],buf[1],buf[2],buf[3],buf[4],
	//	buf[5],buf[6],buf[7]);

	memcpy(&stIOCtrlHead, buf, sizeof(st_AVIOCtrlHead));
	if ((gClientInfo[CID].SID < 0) || (gClientInfo[CID].pBuf == NULL)) {
		printf("Invaild Client\n");
		return;
	}

	printf("Handle_IOCTRL_Cmd : %x\r\n", stIOCtrlHead.nIOCtrlType);

	switch (stIOCtrlHead.nIOCtrlType) {
	case IOCTRL_DEV_LOGIN: {
		SLOGINReq req_login;
		memcpy(&req_login, buf + 4, sizeof(SLOGINReq));
		printf("IOCTRL_DEV_LOGIN(%d):%s SID:%d\n", stIOCtrlHead.nIOCtrlDataSize, req_login.pwd, SID);
		//const char *p2ppasswd = nvram_bufget(RT2860_NVRAM, "P2PPasswd");
		xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
		char *sendBuf = gClientInfo[CID].pBuf ;
		st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
		st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
		SLOGINResp *avStream = (SLOGINResp *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
		ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SLOGINResp);
		ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
		ctrlHead->nIOCtrlType = IOCTRL_DEV_LOGIN ;
		ctrlHead->nIOCtrlDataSize = sizeof(SLOGINResp);

		if (strcmp((char *)req_login.pwd, (char *)wnet_config.user_pwd) == 0) {
			avStream->ret = 0;
			gClientInfo[CID].login = 1;
			printf("Login Success\r\n");
		} else {
			avStream->ret = -1;
			gClientInfo[CID].login = 0;
			printf("Login Fail\r\n");
		}
		avStream->fecResoult = 4;
		avStream->changeInitPwd = 0;
		avStream->numCh = 1;
		avStream->res[0].width = 1920;
		avStream->res[0].height = 1080;
		memset(avStream->version, 0, 16);
		strcpy((char *)avStream->version, "201707151806");
		avStream->mode = 0;
		avStream->light_sw = Light_SW;
		avStream->light_min = Light_Duty;
		avStream->night_light_sw = Night_Light_SW;
		avStream->night_light_min = Night_Light_Duty;
		avStream->motion_sw = Motion_SW;
		avStream->motion_value = Motion_Value;

		avStream->recType = Record_Type;
		nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SLOGINResp)) ;
		xSemaphoreGive(gClientInfo[CID].pBuf_mutex);

	}
	break;
	case IOCTRL_DEV_LOGOUT: {
		printf("IOCTRL_DEV_LOGOUT : %d SID:%d\r\n", Light_Duty, SID);
		xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
		char *sendBuf = gClientInfo[CID].pBuf ;
		st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
		st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
		SReq *avStream = (SReq *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
		ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SReq);
		ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
		ctrlHead->nIOCtrlType = IOCTRL_DEV_LOGOUT ;
		ctrlHead->nIOCtrlDataSize = sizeof(SReq);
		avStream->token = BattCap;
		nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SReq)) ;
		xSemaphoreGive(gClientInfo[CID].pBuf_mutex);
	}
	break;
	case IOCTRL_TYPE_VIDEO_START: {
		SMsgAVIoctrlAVStream req_start;
		memcpy(&req_start, buf + sizeof(st_AVIOCtrlHead), stIOCtrlHead.nIOCtrlDataSize);
		printf("Handle_IOCTRL_Cmd IOCTRL_TYPE_VIDEO_START: %d,%d SID:%d\r\n", CID, req_start.channel, SID);
		regedit_client_to_video(CID);

		xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
		char *sendBuf = gClientInfo[CID].pBuf ;
		st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
		st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
		SMsgAVIoctrlAVStream *avStream = (SMsgAVIoctrlAVStream *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
		ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlAVStream);
		ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
		ctrlHead->nIOCtrlType = IOCTRL_TYPE_VIDEO_START ;
		ctrlHead->nIOCtrlDataSize = sizeof(SMsgAVIoctrlAVStream);
		avStream->channel = 0;
		nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlAVStream)) ;
		xSemaphoreGive(gClientInfo[CID].pBuf_mutex);
	}
	break;
	case IOCTRL_TYPE_VIDEO_STOP: {
		SMsgAVIoctrlAVStream req_stop;
		memcpy(&req_stop, buf + sizeof(st_AVIOCtrlHead), stIOCtrlHead.nIOCtrlDataSize);
		printf("Handle_IOCTRL_Cmd: %d,%d IOCTRL_TYPE_VIDEO_STOP CH:%d SID:%d\r\n", CID, req_stop.channel, SID);

		unregedit_client_from_video(CID);

		xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
		char *sendBuf = gClientInfo[CID].pBuf ;
		st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
		st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
		SMsgAVIoctrlAVStream *avStream = (SMsgAVIoctrlAVStream *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
		ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlAVStream);
		ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
		ctrlHead->nIOCtrlType = IOCTRL_TYPE_VIDEO_STOP ;
		ctrlHead->nIOCtrlDataSize = sizeof(SMsgAVIoctrlAVStream);
		avStream->channel = 0;
		nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlAVStream)) ;
		xSemaphoreGive(gClientInfo[CID].pBuf_mutex);
	}
	break;
	case IOCTRL_TYPE_AUDIO_START: {
		SMsgAVIoctrlAVStream req_start;
		memcpy(&req_start, buf + sizeof(st_AVIOCtrlHead), stIOCtrlHead.nIOCtrlDataSize);
		printf("Handle_IOCTRL_Cmd IOCTRL_TYPE_AUDIO_START: %d,%d SID:%d\r\n", CID, req_start.channel, SID);
		regedit_client_to_audio(CID);

		xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
		char *sendBuf = gClientInfo[CID].pBuf ;
		st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
		st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
		SMsgAVIoctrlAVStream *avStream = (SMsgAVIoctrlAVStream *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
		ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlAVStream);
		ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
		ctrlHead->nIOCtrlType = IOCTRL_TYPE_AUDIO_START ;
		ctrlHead->nIOCtrlDataSize = sizeof(SMsgAVIoctrlAVStream);
		avStream->channel = 0;
		nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlAVStream)) ;
		xSemaphoreGive(gClientInfo[CID].pBuf_mutex);
	}
	break;
	case IOCTRL_TYPE_AUDIO_STOP: {
		SMsgAVIoctrlAVStream req_stop;
		memcpy(&req_stop, buf + sizeof(st_AVIOCtrlHead), stIOCtrlHead.nIOCtrlDataSize);
		printf("Handle_IOCTRL_Cmd: %d,%d IOCTRL_TYPE_AUDIO_STOP CH:%d SID:%d\r\n", CID, req_stop.channel, SID);

		unregedit_client_from_audio(CID);

		xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
		char *sendBuf = gClientInfo[CID].pBuf ;
		st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
		st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
		SMsgAVIoctrlAVStream *avStream = (SMsgAVIoctrlAVStream *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
		ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlAVStream);
		ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
		ctrlHead->nIOCtrlType = IOCTRL_TYPE_AUDIO_STOP ;
		ctrlHead->nIOCtrlDataSize = sizeof(SMsgAVIoctrlAVStream);
		avStream->channel = 0;
		nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlAVStream)) ;
		xSemaphoreGive(gClientInfo[CID].pBuf_mutex);
	}
	break;
	case IOCTRL_TYPE_SPEAKER_START: {
		SMsgAVIoctrlAVStream req_start;
		memcpy(&req_start, buf + sizeof(st_AVIOCtrlHead), stIOCtrlHead.nIOCtrlDataSize);
		printf("Handle_IOCTRL_Cmd IOCTRL_TYPE_SPEAKER_START: %d,%d SID:%d\r\n", CID, req_start.channel, SID);

		xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
		char *sendBuf = gClientInfo[CID].pBuf ;
		st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
		st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
		SMsgAVIoctrlAVStream *avStream = (SMsgAVIoctrlAVStream *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
		ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlAVStream);
		ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
		ctrlHead->nIOCtrlType = IOCTRL_TYPE_SPEAKER_START ;
		ctrlHead->nIOCtrlDataSize = sizeof(SMsgAVIoctrlAVStream);
		avStream->channel = 0;
		nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlAVStream)) ;
		xSemaphoreGive(gClientInfo[CID].pBuf_mutex);
		//f_open(&m_file, "dump_speaker.wav", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		if (speaker_start == 0) {
			speaker_start = 1;
		}
	}
	break;
	case IOCTRL_TYPE_SPEAKER_STOP: {
		SMsgAVIoctrlAVStream req_stop;
		memcpy(&req_stop, buf + sizeof(st_AVIOCtrlHead), stIOCtrlHead.nIOCtrlDataSize);
		printf("Handle_IOCTRL_Cmd: %d,%d IOCTRL_TYPE_SPEAKER_STOP CH:%d SID:%d\r\n", CID, req_stop.channel, SID);

		xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
		char *sendBuf = gClientInfo[CID].pBuf ;
		st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
		st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
		SMsgAVIoctrlAVStream *avStream = (SMsgAVIoctrlAVStream *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
		ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlAVStream);
		ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
		ctrlHead->nIOCtrlType = IOCTRL_TYPE_SPEAKER_STOP ;
		ctrlHead->nIOCtrlDataSize = sizeof(SMsgAVIoctrlAVStream);
		avStream->channel = 0;
		nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlAVStream)) ;
		xSemaphoreGive(gClientInfo[CID].pBuf_mutex);

	}
	break;
	case IOCTRL_IPC_GETRECORD : {
		SIoctrlGetRecordReq req;
		printf("IOCTRL_IPC_GETRECORD\r\n");

		xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
		char *sendBuf = gClientInfo[CID].pBuf ;
		st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
		st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
		SIoctrlGetRecordResp *avStream = (SIoctrlGetRecordResp *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
		ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SIoctrlGetRecordResp);
		ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
		ctrlHead->nIOCtrlType = IOCTRL_TYPE_SPEAKER_STOP ;
		ctrlHead->nIOCtrlDataSize = sizeof(SIoctrlGetRecordResp);
		avStream->ret = 0;
		avStream->channel = 0;
		avStream->recType = wnet_config.record_mode;
		nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SIoctrlGetRecordResp)) ;
		xSemaphoreGive(gClientInfo[CID].pBuf_mutex);
	}
	break;
	case IOTYPE_USER_IPCAM_LAMP: {
		SMsgIoctrlBright req;
		memcpy(&req, buf + sizeof(st_AVIOCtrlHead), stIOCtrlHead.nIOCtrlDataSize);
		printf("IOTYPE_USER_IPCAM_LAMP CMD:%d\r\n", req.cmd);
		if (req.cmd == LAMP_CMD_SET_BRIGHT) {
			printf("LAMP_CMD_SET_BRIGHT SW:%d Value:%d\r\n", req.sw, req.min);
			//gpio_write(&gpio_led, req.sw);
			//Light_SW = req.sw;
		} else if (req.cmd == LAMP_CMD_GET_BRIGHT)  {
			printf("LAMP_CMD_GET_BRIGHT %d SID:%d\r\n", Light_Duty, SID);
			xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
			char *sendBuf = gClientInfo[CID].pBuf ;
			st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
			st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
			SMsgIoctrlBright *avStream = (SMsgIoctrlBright *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
			ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SMsgIoctrlBright);
			ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
			ctrlHead->nIOCtrlType = IOTYPE_USER_IPCAM_LAMP ;
			ctrlHead->nIOCtrlDataSize = sizeof(SMsgIoctrlBright);
			avStream->cmd = LAMP_CMD_GET_BRIGHT;
			avStream->sw = Light_SW;
			avStream->min = 50;
			nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SMsgIoctrlBright)) ;
			xSemaphoreGive(gClientInfo[CID].pBuf_mutex);
			printf("LAMP_CMD_GET_BRIGHT: %d\n", nRet);
		}
	}
	break;
	case IOTYPE_USER_SERIAL_TRANSPARENT: {
		SMsgIoctrlSerial req_msg;
		memcpy(&req_msg, buf + 4, stIOCtrlHead.nIOCtrlDataSize);
		printf("IOTYPE_USER_SERIAL_TRANSPARENT(%d) Meg:%s Len:%d SID:%d\r\n", stIOCtrlHead.nIOCtrlDataSize, req_msg.msg, strlen((const char *)req_msg.msg), SID);
		putSerialPacket((char *)req_msg.msg, strlen((const char *)req_msg.msg));
	}
	break;
	case IOTYPE_USER_IPCAM_LISTEVENT_REQ: {
		st_AVIOCtrlHead r;
		SMsgAVIoctrlListEventReq req;

		memcpy(&req, buf + 4, stIOCtrlHead.nIOCtrlDataSize);
		printf("IOTYPE_USER_IPCAM_LISTEVENT_REQ\r\n");
		printf("stStartTime : %4d/%2d/%2d/%2d:%2d\r\n", req.stStartTime.year, req.stStartTime.month, req.stStartTime.day,  req.stStartTime.hour,
			   req.stStartTime.minute);
		printf("stEndTime : %4d/%2d/%2d/%2d:%2d\r\n", req.stEndTime.year, req.stEndTime.month, req.stEndTime.day, req.stEndTime.hour, req.stEndTime.minute);

		gEventList->total = 1;
		gEventList->index = 0;
		gEventList->endflag = 1;

		//gEventList->count = skynet_list_files("",gEventList->stEvent);


		xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
		char *sendBuf = gClientInfo[CID].pBuf ;
		st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
		st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
		SMsgIoctrlBright *avStream = (SMsgIoctrlBright *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
		ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SMsgIoctrlBright);
		ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
		ctrlHead->nIOCtrlType = IOTYPE_USER_IPCAM_LISTEVENT_RESP ;
		ctrlHead->nIOCtrlDataSize = sizeof(SMsgAVIoctrlListEventResp) + sizeof(SAvEvent) * gEventList->count;
		memcpy(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead), gEventList, sizeof(SMsgAVIoctrlListEventResp) + sizeof(SAvEvent)*gEventList->count);

		nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlListEventResp) + sizeof(
							   SAvEvent) * gEventList->count) ;
		xSemaphoreGive(gClientInfo[CID].pBuf_mutex);

	}
	break;
	case IOTYPE_USER_IPCAM_RECORD_PLAYCONTROL: {
		st_AVIOCtrlHead r;
		SMsgAVIoctrlPlayRecordResp res;
		memcpy(&pb_req, buf + 4, stIOCtrlHead.nIOCtrlDataSize);
		printf("IOTYPE_USER_IPCAM_RECORD_PLAYCONTROL cmd[%d]\r\n", pb_req.command);
		printf("p->stTimeDay : %04d/%02d/%02d %02d:%02d:%02d\r\n", pb_req.stTimeDay.year, pb_req.stTimeDay.month, pb_req.stTimeDay.day, pb_req.stTimeDay.hour,
			   pb_req.stTimeDay.minute, pb_req.stTimeDay.second);
		if (pb_req.command == AVIOCTRL_RECORD_PLAY_START) {
			printf("AVIOCTRL_RECORD_PLAY_START : %d\r\n", pb_req.dummy);

			xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
			char *sendBuf = gClientInfo[CID].pBuf ;
			st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
			st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
			SMsgAVIoctrlPlayRecordResp *avStream = (SMsgAVIoctrlPlayRecordResp *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
			ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlPlayRecordResp);
			ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
			ctrlHead->nIOCtrlType = IOTYPE_USER_IPCAM_RECORD_PLAYCONTROL_RESP ;
			ctrlHead->nIOCtrlDataSize = sizeof(SMsgAVIoctrlPlayRecordResp);
			avStream->command = AVIOCTRL_RECORD_PLAY_START;
			avStream->result = 0;
			avStream->duration = 21;
			nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlPlayRecordResp)) ;
			xSemaphoreGive(gClientInfo[CID].pBuf_mutex);
			PB_CID = CID;
			PB_SID = SID;
			memset(pb_file_path, 0, 64);

		} else if (pb_req.command == AVIOCTRL_RECORD_PLAY_PAUSE) {
			printf("AVIOCTRL_RECORD_PLAY_PAUSE\r\n");
			gClientInfo[CID].bPausePlayBack = !gClientInfo[CID].bPausePlayBack;
			xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
			char *sendBuf = gClientInfo[CID].pBuf ;
			st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
			st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
			SMsgAVIoctrlPlayRecordResp *avStream = (SMsgAVIoctrlPlayRecordResp *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
			ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlPlayRecordResp);
			ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
			ctrlHead->nIOCtrlType = IOTYPE_USER_IPCAM_RECORD_PLAYCONTROL_RESP ;
			ctrlHead->nIOCtrlDataSize = sizeof(SMsgAVIoctrlPlayRecordResp);
			avStream->command = AVIOCTRL_RECORD_PLAY_PAUSE;
			avStream->result = 0;
			nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlPlayRecordResp)) ;
			xSemaphoreGive(gClientInfo[CID].pBuf_mutex);
		} else if (pb_req.command == AVIOCTRL_RECORD_PLAY_STOP) {
			printf("AVIOCTRL_RECORD_PLAY_STOP\r\n");
			gClientInfo[CID].bStopPlayBack = 1;
			xSemaphoreTake(gClientInfo[CID].pBuf_mutex, portMAX_DELAY);
			char *sendBuf = gClientInfo[CID].pBuf ;
			st_AVStreamIOHead *ioHead = (st_AVStreamIOHead *)(sendBuf);
			st_AVIOCtrlHead *ctrlHead = (st_AVIOCtrlHead *)(sendBuf + sizeof(st_AVStreamIOHead));
			SMsgAVIoctrlPlayRecordResp *avStream = (SMsgAVIoctrlPlayRecordResp *)(sendBuf + sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead));
			ioHead->nStreamIOHead = sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlPlayRecordResp);
			ioHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_IOCTRL;
			ctrlHead->nIOCtrlType = IOTYPE_USER_IPCAM_RECORD_PLAYCONTROL_RESP ;
			ctrlHead->nIOCtrlDataSize = sizeof(SMsgAVIoctrlPlayRecordResp);
			avStream->command = AVIOCTRL_RECORD_PLAY_STOP;
			avStream->result = 0;
			nRet = SKYNET_send(SID, sendBuf, sizeof(st_AVStreamIOHead) + sizeof(st_AVIOCtrlHead) + sizeof(SMsgAVIoctrlPlayRecordResp)) ;
			xSemaphoreGive(gClientInfo[CID].pBuf_mutex);
		} else if (pb_req.command == AVIOCTRL_RECORD_PLAY_SEEKTIME) {
			printf("AVIOCTRL_RECORD_PLAY_SEEKTIME\r\n");
		}
	}
	break;
	}
}

void task_ListenSession(void *param)
{
	if (sky_init_flag == 1) {
		printf("\r\n***********Deinit - 1*********\r\n");
		resetClients();
		printf("\r\n***********Deinit - 2*********\r\n");
		//vTaskDelay(5000);
		skyNetModuleDeinit();
		printf("\r\n***********Deinit - 3*********\r\n");
		//vTaskDelay(5000);
	}
	skyNetModuleInit();
	skyNetModuleListen();
	//vTaskDelay(200);
	vTaskDelete(NULL);
}


void task_CheckSession(void *param)
{
	printf("[task_CheckSession] start running....\r\n");
	int cCID = -1;
	int sleep_cnt = 0;

	while (gProcessRun) {
		if ((cCID = checkDestoryClient()) >= 0) {
			printf("\r\n@@@@@@@@@@@ %d @@@@@@@@@@\r\n", cCID);
			//printf("freeClient\n");
			freeClient(cCID);
		}

		if (ghostSocketID > 0) {
			printf("\r\n++++++++++ Close Ghost %d ++++++++++\r\n", ghostSocketID);
			SKYNET_close(ghostSocketID, HOW_SHUT_RDWR);
			ghostSocketID = -1;
		}

		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
	printf("[task_CheckSession] end....\r\n");
	vTaskDelete(NULL);
}

void task_HandleSession(void *param)
{
	printf("[task_HandleSession] start running....\r\n");
	int CID = -1;
	int nRet;
	int SID;
	char cmdBuf[256];

	while (gProcessRun) {
		if (UID_Setup == 1) {
			UID_Setup = 0;
			printf("UID Write : %s\n", gP2PString);
			Save_P2P_Config();
			Load_P2P_Config();
			UID_Reset = 1;
		}

		if (Config_Reset == 1) {
			Config_Reset = 0;
			printf("Config_Reset\n");
		}

		memset(cmdBuf, 0, 256);
		if ((nRet = getCMDPacket(&SID, &CID, cmdBuf)) > 0) {
			Handle_IOCTRL_Cmd(SID, CID, cmdBuf);
		}

		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
	printf("[task_HandleSession] end....\r\n");
	vTaskDelete(NULL);
}

void test_max_socket(void)
{
	printf("\r\n================test_max_socket==================\r\n");
	while (1) {
		int n = socket(PF_INET,  SOCK_STREAM, 0);
		if (n >= 0) {
			printf("socket open success n = %d\r\n", n);
		} else {
			printf("socket failed n = %d, errno = %d\r\n", n, errno);
			break;
		}
	}
}

int getIPNotice = 0;

void task_LoginSession(void *param)
{
	printf("[task_LoginSession] start running ....\r\n");

	while (gProcessRun) {
		if ((getIPNotice == 1) || (startAPMode == 1) || (UID_Reset == 1)) {
			getIPNotice = 0;
			loginFail = 0;
			startAPMode = 0;
			UID_Reset = 0;
			printf("\r\n****************Get IP!!!(%d)****************\r\n", sky_init_flag);

			if (sky_init_flag == 1) {
				printf("\r\n***********Deinit - 1*********\r\n");
				resetClients();
				printf("\r\n***********Deinit - 2*********\r\n");
				//vTaskDelay(5000);
				skyNetModuleDeinit();
				printf("\r\n***********Deinit - 3*********\r\n");
				//vTaskDelay(5000);
			}

			skyNetModuleInit();

			skyNetModuleListen();

			sntp_init();

			//check_internet();

		}

		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
	printf("[task_LoginSession] end ....\r\n");
	vTaskDelete(NULL);
}


extern int tx_mode;
//#include "audio_tool_command.h"
extern mm_context_t	*audio_save_ctx;
void task_audio_pcm(void *param)
{
	while (1) {
		mm_context_t *ctx = (mm_context_t *)audio_save_ctx;
		//printf("audio_output_item.flag=%d && tx_mode =%d\r\n", audio_output_item.flag, tx_mode);
		if (audio_output_item.flag == 1 && tx_mode == 0 && gProcessRun) {
			audio_handle(ctx->priv, &audio_output_item, NULL);
			audio_output_item.flag = 0;
		}
		vTaskDelay(1);
	}
}

void deinitAVInfo(void)
{
	int i, j;
	for (i = 0; i < MAX_CLIENT_NUMBER; i++) {
		memset(&(gClientInfo[i]), 0, sizeof(AV_Client));
		gClientInfo[i].SID = -1;
		gClientInfo[i].destroyFlag = 1;
		printf("Deiniital gClientInfo[i].pBuf_mutex\r\n");
		xSemaphoreTake(gClientInfo[i].pBuf_mutex, portMAX_DELAY);
		vSemaphoreDelete(gClientInfo[i].pBuf_mutex);
	}

	vTaskDelay(1000);
	printf("Deintial gClientInfo[i].pBuf_mutex\r\n");
	//gSID_mutex = xSemaphoreCreateMutex();
	//cmd_q_mutex = xSemaphoreCreateMutex();
	//serial_q_mutex = xSemaphoreCreateMutex();
	vSemaphoreDelete(gSID_mutex);
	vSemaphoreDelete(cmd_q_mutex);
	vSemaphoreDelete(serial_q_mutex);


	//pQ = ConstructQueue(7);
	DestructQueue(pQ);
	pQ = NULL;
	DestructQueue(pQ_serial);
	pQ_serial = NULL;
	free(gEventList);
	//gEventList = (SMsgAVIoctrlListEventResp *)malloc(sizeof(SMsgAVIoctrlListEventResp) + sizeof(SAvEvent) * MAX_LISTEVENT_COUNT);
	//memset(gEventList, 0, sizeof(SMsgAVIoctrlListEventResp));

	memset(gUID, 0, 25);
	memset(gKey, 0, 8);
	memset(gPass, 0, 16);

	//Load_P2P_Config();

	//sprintf(wnet_config.user_pwd, "12345678");
}

int audio_task_first = 0;
void skynet_device_run(void)
{
	initAVInfo();
	if (audio_task_first == 0) {

		if (xTaskCreate(task_audio_pcm, ((const char *)"task_audio_pcm"), 4096  + 2048, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
			printf("Create task_audio_pcm error!!!!\n");
		}
		audio_task_first = 1;
	}
	printf("\r\n(((((((((((((((((((((skynet_device_run))))))))))))))))))))))))\r\n");

	if (xTaskCreate(task_HandleSession, ((const char *)"SkynetDev"), 2048, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
		printf("Create task_HandleSession error!!!!\n");
	}

	if (xTaskCreate(task_CheckSession, ((const char *)"SkynetCheck"), 2048, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
		printf("Create task_CheckSession error!!!!\n");
	}

	if (xTaskCreate(task_LoginSession, ((const char *)"task_LoginSession"), 2048, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
		printf("Create task_LoginSession error!!!!\n");
	}
}
