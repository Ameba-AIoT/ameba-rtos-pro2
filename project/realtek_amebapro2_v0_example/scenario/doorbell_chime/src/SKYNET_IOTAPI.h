/*
 * library of iot native API header
 *
 * Copyright (c) 2016 SKYNET Co., Ltd.
 * All rights reserved.
 *
 */

#ifndef SKYNET__IOTAPI_H__
#define SKYNET__IOTAPI_H__

#if !defined (_WIN32)
#define __stdcall
#endif

#define SKYNET_IOT_MODULE_VERSION 0x01050400

/** The time Device or Client to send session keepalive packet to remote site. */
#define SKYNET_IOT_SESSION_KEEPALIVE_INTERVAL_SECOND 5

/** When reach time(SKYNET_IOT_SESSION_KEEPALIVE_TIMEOUT_THRESHOLD * SKYNET_IOT_SESSION_KEEPALIVE_INTERVAL_SECOND) not receive
	any alive packet will regard as session connection broken.
 */
#define SKYNET_IOT_SESSION_KEEPALIVE_TIMEOUT_THRESHOLD 6

/** The time Device to send login keepalive packet to ICE Server. */
#define SKYNET_IOT_LOGIN_KEEPALIVE_INTERVAL_SECOND 55

/** When reach time(SKYNET_IOT_LOGIN_KEEPALIVE_TIMEOUT_THRESHOLD * SKYNET_IOT_LOGIN_KEEPALIVE_INTERVAL_SECOND) not receive
	any login response will regard as TCP connection broken.
 */
#define SKYNET_IOT_LOGIN_KEEPALIVE_TIMEOUT_THRESHOLD 2

/** Device and Client give complete ID to API, and LAN search will get this.
 */
#define SKYNET_IOT_MAX_COMPLETE_ID_LEN 28

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** Get module version
	@param nMode Decide get which module version, 1<<1: IOT module
	@return NULL if give wrong module mode, or get a string to specify version ex: 1.2.3.4
*/
const char *SKYNET_Module_Version(int nMode);

/** Initialize module resource
	@param nMode Decide initialize which module, 1<<1: IOT module
	@return 0 if successful, or <0 if an error occurred and please refer to error code for detail
*/
int SKYNET_Module_Init(int nMode);

/** Deinitialize module resource
	@param nMode Decide deinitialize which module, 1<<1: IOT module
	@return 0 if successful, or <0 if an error occurred and please refer to error code for detail
*/
int SKYNET_Module_Deinit(int nMode);

#define IOT_RECV_WANT_CONTINUE_TO_READ 20
#define IOT_RECV_ALREADY_GOT_COMPLETE_DATA 30
#define IOT_RECV_NO_DATA_CAN_READ -10
#define IOT_RECV_ERROR_OCCURRED -1
/** Prototype of callback function
	@param nSocketID Which connected socket number
	@param nSize How many data size readable, or <0 means session error occurred
	@param pUserData The data given by register callback function
	@return IOT_RECV_WANT_CONTINUE_TO_READ if want to continue to read data
			IOT_RECV_ALREADY_GOT_COMPLETE_DATA if already finished to read data
			IOT_RECV_NO_DATA_CAN_READ if now not data to read, ex: call SKYNET_recv() return is 0
			IOT_RECV_ERROR_OCCURRED if call SKYNET_recv() return <0 must return this define code
	NOTE: The function you provide *must not* do blocking behavior.
          Doing so will let library come to a halt.
*/
typedef int (__stdcall *socketRecvCB)(int nSocketID, int nSize, void *pUserData);
/** Register callback function and when data coming will invoke this callback function
	@param pfxRecvCb The callback function
	@param pUserData When callback function invoked this data will pass to callback function
*/
void SKYNET_set_recv_callback(socketRecvCB pfxRecvCb, void *pUserData);

/** Open a new socket resource
	@param nLocalPort 0 use a random port determined by OS, other value range is 1~65535
	@return >=0 if successful, or <0 if an error occurred and please refer to error code for detail
*/
int SKYNET_socket(unsigned short nLocalPort);

#define SKYNET_MAX_DEVICE_UUID_LEN			63
/** Set the UUID of device, is the unique ID of product
	@param pszUUID Device ID, max. length is SKYNET_MAX_DEVICE_UUID_LEN
	NOTE: Please call it after SKYNET_Module_Init()
*/
void SKYNET_set_UUID(const char *pszUUID);

typedef struct _st_HostIpInfo st_HostIpInfo;
struct _st_HostIpInfo {
	char strCountryCode[3]; // 2 character and null-terminated
	char strRegionCode[5]; // 4 character and null-terminated
	char strLatitude[11]; // max. 10 character and null-terminated
	char strLongitude[11]; // max. 10 character and null-terminated
	char padding[2];
};
/** This function will set host ip information and then send it to SEED server for record
	@param pstHostIpInfo Refer to struct st_HostIpInfo
	NOTE: Must call it before SKYNET_listen()
*/
void SKYNET_SetHostIpInfo(st_HostIpInfo *pstHostIpInfo);

/** Prototype of callback function
	@param nSocketID Which listened socket number
	@param nResult 0 if successful, or <0 if an error occurred and please refer to error code for detail
	@param pUserData The data given by register callback function
*/
typedef void (__stdcall *loginCB)(int nSocketID, int nResult, void *pUserData);
/** Register callback function and when login status changed will invoke this callback function
	@param pfxLoginCb The callback function
	@param pUserData When callback function invoked this data will pass to callback function
*/
void SKYNET_set_login_callback(loginCB pfxLoginCb, void *pUserData);

typedef enum {
	DEVICETYPE_UNKNOWN = 0,
	DEVICETYPE_IPCAM = 1,
	DEVICETYPE_CAR_DV = 2,
	DEVICETYPE_DOORBELL = 3,
	DEVICETYPE_BABY_MONITOR = 4,
	DEVICETYPE_SPORT_DV = 5,
	DEVICETYPE_ACTION_CAM = 6,
	DEVICETYPE_WIFI_CAM = 7,
	DEVICETYPE_BULB_CAM = 8,
	DEVICETYPE_360_CAM = 9,
	DEVICETYPE_FISH_TANK_CAM = 10,
	DEVICETYPE_POLICE_CAM = 11
} en_DeviceType;

typedef enum {
	ICTYPE_UNKNOWN = 0,
	ICTYPE_REALTEK = 1,
	ICTYPE_NOVATEK = 2,
	ICTYPE_GM = 3,
	ICTYPE_SONIX = 4,
	ICTYPE_VATICS = 5,
	ICTYPE_TI = 6,
	ICTYPE_POLIFIC = 7,
	ICTYPE_ALLWINNER = 8,
	ICTYPE_ROCKCHIP = 9,
	ICTYPE_INFOTM = 10,
	ICTYPE_AMBARELLA = 11,
	ICTYPE_HISILLICON = 12,
	ICTYPE_BROADCOM = 13,
	ICTYPE_QUALCOMM = 14,
	ICTYPE_AIT = 15,
	ICTYPE_INGENIC = 16,
	ICTYPE_GENERALPLUS = 17,
	ICTYPE_MTK = 18
} en_IcType;

typedef enum {
	OSTYPE_UNKNOWN = 0,
	OSTYPE_LINUX = 1,
	OSTYPE_IOS = 2,
	OSTYPE_ANDROID = 3,
	OSTYPE_WINDOWS = 4,
	OSTYPE_FREERTOS = 5,
	OSTYPE_THREADX = 6,
	OSTYPE_ECOS = 7,
	OSTYPE_NUCLEUS = 8,
	OSTYPE_ITRON = 9,
	OSTYPE_RTLINUX = 10,
	OSTYPE_VXWORKS = 11
} en_OsType;

#define SKYNET_MAX_WAKEUP_REASON_LEN		32

typedef struct _st_DeviceInfo st_DeviceInfo;
struct _st_DeviceInfo {
	/** For device supported sleep mode, you can assign wakeup reason and it will record in Server log.
	 *  Note: Not use ";" to assign content to the reason because Server log use ";" for separating log section.
	*/
	char strWakeupReason[SKYNET_MAX_WAKEUP_REASON_LEN];
	en_DeviceType eDeviceType; // refer to en_DeviceType define
	en_IcType eIcType; // refer to en_IcType define
	en_OsType eOsType; // refer to en_OsType define
	unsigned int nOsVersion; // OS version
	unsigned int isEnableWakeupFunction; // 1: let Server know this device own wakeup capacity and enable remote wakeup flow
};

#define SKYNET_MAX_DEVICE_ID_LEN			24
#define SKYNET_MAX_DEVICE_USER_KEY_LEN		6
#define SKYNET_MAX_DEVICE_TYPE_LEN			64
/** Let the socket in listening state and it will login to ICE server
	@param nSocketID It got from SKYNET_socket()
	@param pszID Device ID, max. length is SKYNET_MAX_DEVICE_ID_LEN
	@param pszUserKey The key defined by customer to match device ID, max. length is SKYNET_MAX_DEVICE_USER_KEY_LEN
	@param pszType Device type for customer to specify product information, max. length is SKYNET_MAX_DEVICE_TYPE_LEN, it can be NULL or string type
	@param nMaxConnection Allow how many client to connect
	@param stDeviceInfo The information of device
	@return 0 if successful, or <0 if an error occurred and please refer to error code for detail
*/
int SKYNET_listen(int nSocketID, const char *pszID, const char *pszUserKey, const char *pszType, unsigned short nMaxConnection, st_DeviceInfo *pstDeviceInfo);

/** It can change device type at any time
	@param pszID Device ID, max. length is SKYNET_MAX_DEVICE_ID_LEN
	@param pszType Device type for customer to specify product information, max. length is SKYNET_MAX_DEVICE_TYPE_LEN, it can be NULL or string type
	@return 0 if successful, or <0 if an error occurred and please refer to error code for detail
*/
int SKYNET_change_device_type(const char *pszID, const char *pszType);

/** Check if new connection established
	@param nSocketID It got from SKYNET_socket() and already in listening state
	@param nTimeoutMs Wait how many millisecond for new connection or give 0 will return immediately
	@return >0 if successful will return a new connected socket, or <0 if an error occurred and please refer to error code for detail
*/
int SKYNET_accept(int nSocketID, unsigned int nTimeoutMs);

/** Prototype of callback function
	@param pszID Which device ID already connected
	@param nSocketID Which socket number already connected
	@param nResult >0 means connect successfully, and 1: Device, 2: Client, it's the same with "Role" member of st_ConnectionInfo
				   <0 if an error occurred and please refer to error code for detail
	@param pUserData The data given by register callback function
	NOTE: The function you provided *must not* do blocking behavior.
          Doing so will let library come to a halt.
*/
typedef void (__stdcall *socketConnectCB)(const char *pszID, int nSocketID, int nResult, void *pUserData);
/** Register callback function and when connection established will invoke this callback function
	@param pfxConnectCb The callback function
	@param pUserData When callback function invoked this data will pass to callback function
*/
void SKYNET_set_connect_callback(socketConnectCB pfxConnectCb, void *pUserData);

/** Let the socket in connecting state and it will connect to remote device
	@param pszID Target device ID, max. length is SKYNET_MAX_DEVICE_ID_LEN
	@param nSocketID It got from SKYNET_socket()
	@param nSkipTrySymmetric if >0 means module will not try P2P when device or client behind symmetric NAT
	@param nForceRelay if >0 means module will not try P2P connection only using relay mode to do connection
	@return 0 if successful but maybe not connected yet, or <0 if an error occurred and please refer to error code for detail
	@see SKYNET_getsockopt() to get the connection status
*/
int SKYNET_connect(const char *pszID, int nSocketID, int nSkipTrySymmetric, int nForceRelay);

/** Device is online or not keepalive timeout with ICE server yte */
#define SKYNET_STATUS_DEVICE_ONLINE 1
/** Device is in sleep mode and or not keepalive timeout with ICE server yet */
#define SKYNET_STATUS_DEVICE_SLEEP  2
/** Prototype of callback function
	@param pszID Which device ID already connected
	@param nSocketID Which socket number already connected
	@param nResult >0 means successful, and 1: Device is online, 2: Device is in sleep mode
				   <0 if an error occurred and please refer to error code for detail
	@param pUserData The data given by register callback function
	NOTE: The function you provided *must not* do blocking behavior.
          Doing so will let library come to a halt.
*/
typedef void (__stdcall *queryStatusCB)(const char *pszID, int nSocketID, int nResult, void *pUserData);
/** Register callback function and when got query status result will invoke this callback function
	@param pfxQueryCb The callback function
	@param pUserData  When callback function invoked this data will pass to callback function
*/
void SKYNET_set_query_status_callback(queryStatusCB pfxQueryCb, void *pUserData);

/** Let the socket in querying state and it will query server for the device ID status
	@param pszID Target device ID, max. length is SKYNET_MAX_DEVICE_ID_LEN
	@param nSocketID It got from SKYNET_socket()
	@param nTimeoutMs Can't be less than 1000, 3000~5000 is a usual value
	@return 0 if successful but not got status yet, or <0 if an error occurred and please refer to error code for detail
	NOTE: When got device status it will report to user from "queryStatusCB" function,
          then check the argument "nResult" to know device status
          After used this API must call SKYNET_close() to release resource
*/
int SKYNET_query_status(const char *pszID, int nSocketID, int nTimeoutMs);

/** Alloc buffer memory for send and receive data
	@param nSocketID It got from SKYNET_socket()
	@param pBuf It will get the useable memory address and use it to call SKYNET_send()
	@param nBufSize How many data size will be used
	@return 0 if successful, or <0 if an error occurred and please refer to error code for detail
	NOTE: If duplicate call it with the same nSocketID,
	      the old buffer will be freed first and then alloc new one.
	      Module will free it when session release or call SKYNET_Module_Deinit(),
	      you "must not" free it.
*/
int SKYNET_alloc_buf(int nSocketID, char **pBuf, int nBufSize);

/** Send data to remote device or client
	@param nSocketID It got from SKYNET_socket() and already connected to remote
	@param pBuf The data buffer will be sent to remote
	@param nBufSize The real size of data buffer
	@return >0 if successful but must note that return size if matched with nBufSize like TCP, or <0 if an error occurred and please refer to error code for detail
	NOTE: Must use the memory address got from SKYNET_alloc_buf() as "pBuf" argument.
*/
int SKYNET_send(int nSocketID, const char *pBuf, int nBufSize);

/** Recvive data from remote device or client
	@param nSocketID It got from SKYNET_socket() and already connected to remote
	@param pBuf The data buffer will be saved data of remote
	@param nBufSize The real size of data buffer
	@return >0 means read how many data length, or <0 if an error occurred and please refer to error code for detail
*/
int SKYNET_recv(int nSocketID, char *pBuf, int nBufSize);

typedef enum {
	CONN_NO = 0,
	CONN_TCP_LAN = 1,
	CONN_TCP_STUN = 2,
	CONN_TCP_TURN = 3,
	CONN_UDP_LAN = 11,
	CONN_UDP_STUN = 12,
	CONN_UDP_TURN = 13
} en_ConnectionMode;

#define SKYNET_XTEA_DEFAULT_ROUND 64
typedef enum {
	ENC_OFF = 0,
	ENC_XTEA_ECB = 1,
	ENC_XTEA_CBC = 2,
	ENC_AES = 3
} en_EncryptionMode;

typedef enum {
	NAT_UNKNOWN = 0,
	NAT_FULL_CONE = 1,
	NAT_RESTRICTED_CONE = 2,
	NAT_SYMMETRIC_DETECTABLE = 3,
	NAT_SYMMETRIC_RANDOM = 4,
	NAT_REUSE_PORT_BLOCKED = 5,
	NAT_UDP_BLOCKED = 7,
	NAT_PORT_CHANGED = 8,
	NAT_IP_CHANGED = 9
} en_NATType;

/** Which combo NAT type with Device and Client the IOT module will do try P2P punch */
#define SKYNET_IOT_TRY_P2P_NAT_TYPE_COMBO_NUM 6

/** How many seconds the IOT module will try P2P punch */
#define SKYNET_IOT_TRY_P2P_TIMEOUT_MILLISECOND 3000

/** How many seconds the IOT module will try P2P punch if device or client behind symmetric NAT */
#define SKYNET_IOT_TRY_P2P_WITH_SYMMETRIC_NAT_TIMEOUT_MILLISECOND 5000

typedef struct _st_ConnectionInfo st_ConnectionInfo;
struct _st_ConnectionInfo {
	char Status; // 0: not connected yet, <0: other error code reference
	unsigned char
	ConnMode; // refer to en_ConnectionMode define // 0: not connected yet, 1: TCP LAN, 2: TCP STUN, 3: TCP TURN, 11: UDP LAN, 12: UDP STUN, 13, UDP TURN
	unsigned char EncMode; // refer to en_EncryptionMode define
	unsigned char RemoteNatType; // refer to en_NATType define
	unsigned char LocalNatType; // refer to en_NATType define
	unsigned char RemoteForceRelay;
	char Padding[2];
	char RemoteIP[16]; // IP of connected remote site
	char WanIP[16]; // My WAN IP, it's used to establish connection
	unsigned short RemotePort; // Port of connected remote site
	unsigned short WanPort; // My WAN port, it's used to establish connection
	char LocalIP[16]; // My LAN IP, it's used to establish connection
	unsigned int RemoteVersion; // IOT module version of remote site
	unsigned short LocalPort; // My LAN port, it's used to establish connection
	unsigned char GotData;
	unsigned char Role; // 1: Device, 2: Client
};

typedef struct _st_LoginInfo st_LoginInfo;
struct _st_LoginInfo {
	char Status; // 0: no result now, 1: login ok, other error code reference
	unsigned short nFailedCnt; // means send how many login packet to Server and not got response
	unsigned char LocalNatType; // refer to en_NATType define
};

typedef struct _st_GetBufferInfo st_GetBufferInfo;
struct _st_GetBufferInfo {
	unsigned int nSendSize; // to get OS socket send buffer
	unsigned int nRecvSize; // to get OS socket recv buffer
};

#define SKYNET_IOT_WAKEUP_TOKEN_LEN  40
#define SKYNET_IOT_WAKEUP_SERVER_MAX_NUMBER  2
#define SKYNET_IOT_WAKEUP_SERVER_MAX_IP_LEN  16
#define SKYNET_IOT_WAKEUP_SERVER_PORT 80
typedef char st_WakeupServerIP[SKYNET_IOT_WAKEUP_SERVER_MAX_IP_LEN]; // null-terminated
typedef struct _st_GetWakeupInfo st_GetWakeupInfo;
struct _st_GetWakeupInfo {
	unsigned int nServerNumber; // how many server support wakeup function
	char WakeupToken[SKYNET_IOT_WAKEUP_SERVER_MAX_NUMBER][SKYNET_IOT_WAKEUP_TOKEN_LEN + 4];
	st_WakeupServerIP ServerIP[SKYNET_IOT_WAKEUP_SERVER_MAX_NUMBER];
};

typedef enum {
	SOCKOPT_LOGIN_INFO = 1,      // related to struct st_LoginInfo
	SOCKOPT_CONNECTION_INFO = 2, // related to struct st_ConnectionInfo
	SOCKOPT_GET_BUFFER_SIZE = 3, // related to struct st_GetBufferInfo, must operate a connected session
	SOCKOPT_GET_WAKEUP_INFO = 4  // related to struct st_GetWakeupInfo, must operate a logined session
} en_SockGetOptMode;

/** This function own manipulate options for the socket, there is many mode
	@param nSocketID It got from SKYNET_socket()
	@param mode Refer to en_SockGetOptMode define
	@param pInfo There is different type relate to different mode
	@return 0 if successful, or <0 if an error occurred and please refer to error code for detail
*/
int SKYNET_getsockopt(int nSocketID, int mode, void *pInfo);

typedef struct _st_SetBufferInfo st_SetBufferInfo;
struct _st_SetBufferInfo {
	unsigned int nSendSize; // to setup OS socket send buffer, the maximum and minimum value is up to OS specification
	// if give 0 means not setup as default
	unsigned int nRecvSize; // to setup OS socket recv buffer, the maximum and minimum value is up to OS specification
	// if give 0 means not setup as default
};

typedef enum {
	SOCKOPT_SET_BUFFER_SIZE = 1, // related to struct st_SetBufferInfo, must operate a connected session
} en_SockSetOptMode;

/** This function own manipulate options for the socket, there is many mode
	@param nSocketID It got from SKYNET_socket() and connected status
	@param mode Refer to en_SockSetOptMode define
	@param pInfo There is different type relate to different mode
	@return 0 if successful, or <0 if an error occurred and please refer to error code for detail
*/
int SKYNET_setsockopt(int nSocketID, int mode, void *pInfo);

typedef struct _st_LanDiscoverInfo st_LanDiscoverInfo;
struct _st_LanDiscoverInfo {
	unsigned int ModuleVersion;
	char ID[SKYNET_IOT_MAX_COMPLETE_ID_LEN];
	char IP[16];
	char strTypeData[SKYNET_MAX_DEVICE_TYPE_LEN];
};
/** Prototype of callback function
	@param nResult 0 means got device info, -1 means reach timeout value
	@param stInfo The information of device found
*/
typedef void (__stdcall *lanDiscoverCB)(int nResult, st_LanDiscoverInfo stInfo);
typedef struct _st_LanDiscoverSetting st_LanDiscoverSetting;
struct _st_LanDiscoverSetting {
	lanDiscoverCB pfxLanDiscoverCb; // can't be NULL, when found device module will invoke this callback function
	unsigned int nSendInterval; // how many millisecond to send broadcast once
};

/** This function will do LAN discover for device
	@param stSetting Refer to struct st_LanDiscoverSetting
	@return 0 if successful, or <0 if an error occurred and please refer to error code for detail
	NOTE: This process will not stop until call SKYNET_lanDiscoverStop()
*/
int SKYNET_lanDiscoverStart(st_LanDiscoverSetting stSetting);

/** This function will stop the LAN discover process
 *  @return 0 if successful, or <0 if an error occurred and please refer to error code for detail
*/
int SKYNET_lanDiscoverStop(void);

typedef enum {
	HOW_SHUT = 0, // equal to only call close() of system
	HOW_SHUT_RD = 1, // call shutdown() with SHUT_RD first and then call close()
	HOW_SHUT_WR = 2, // call shutdown() with SHUT_WR first and then call close()
	HOW_SHUT_RDWR = 3, // call shutdown() with SHUT_RDWR first and then call close()
	HOW_SHUT_INNER_USE = 10 // module inner call, use must not use this
} en_SockCloseMethod;

/** This function own manipulate options for the socket, there is many mode
	@param nSocketID It got from SKYNET_socket()
	@param how Refer to en_SockCloseMethod define
	@return 0 if successful, or <0 if an error occurred and please refer to error code for detail
*/
int SKYNET_close(int nSocketID, int how);

/** This function can setup debug log file path and name
	@param pszFileName Must give full path and file name
	NOTE: Please call it before SKYNET_Module_Init() to log complete message
	NOTE: Maximum file name length is 256
*/
void SKYNET_SetLogPath(const char *pszFileName);

/** This function will let module only use relay mode method to do connection
	NOTE: Please call it after SKYNET_Module_Init()
*/
void SKYNET_ForceRelayMode(void);

/** This function will let module not bind the same local port to connect with server and device
	NOTE: Please call it after SKYNET_Module_Init()
*/
void SKYNET_NotReuseLocalPort(void);

/** Device is online or not keepalive timeout with ICE server yte */
#define SKYNET_STATUS_SET_ID_REGION_OK 1
/** Prototype of callback function
	@param pszID Which device ID already connected
	@param nSocketID Which socket number already connected
	@param nResult >0 means successful, and 1: set id region is ok and the device can relogin to assigned ICE server
				   <0 if an error occurred and please refer to error code for detail
	@param pUserData The data given by register callback function
	NOTE: The function you provided *must not* do blocking behavior.
          Doing so will let library come to a halt.
*/
typedef void (__stdcall *setIDRegionCB)(const char *pszID, int nSocketID, int nResult, void *pUserData);
/** Register callback function and when got set id region result will invoke this callback function
	@param pfxSetIDRegionCb The callback function
	@param pUserData  When callback function invoked this data will pass to callback function
*/
void SKYNET_set_id_region_callback(setIDRegionCB pfxSetIDRegionCb, void *pUserData);
typedef enum {
	REGION_NONE = 0,  // It means to reset id to default region
	REGION_CN   = 1,  // China
	REGION_N_CN = 2,  // North of China
	REGION_S_CN = 3,  // South of China
	REGION_US   = 10, // America
	REGION_W_US = 11, // West of America
	REGION_E_US = 12, // East of America
	REGION_EU   = 20, // Europe
	REGION_W_EU = 21, // West of Europe
	REGION_E_EU = 22, // East of Europe
	REGION_JP   = 30, // Japan
	REGION_KR   = 40, // Korea
	REGION_SG   = 50, // Singapore
	REGION_AU   = 60, // Australia
	REGION_BR   = 70, // Brazil
	REGION_RU   = 80, // Russia
} en_IDRegion;
/** This function own manipulate options for the socket, there is many mode
	@param pszID Target device ID, max. length is SKYNET_MAX_DEVICE_ID_LEN
	@param nSocketID It got from SKYNET_socket()
	@param nRegion Refer to en_IDRegion define
	@return 0 if successful, or <0 if an error occurred and please refer to error code for detail
*/
int SKYNET_set_id_region(const char *pszID, int nSocketID, int nRegion);

#if defined(PLATFORM_FREERTOS)
/** Prototype of callback function
	@param nSocketID The tcp socket connected with server
	@param nResult 0 if successful, or <0 if an error occurred and please refer to error code for detail
	@param pUserData The data given by register callback function
*/
typedef void (__stdcall *wakeupAuthCB)(int nSocketID, int nResult, void *pUserData);
/** Register callback function and when server responded wakeup authentication result will invoke this callback function
	@param pfxWakeupAuthCb The callback function
	@param pUserData When callback function invoked this data will pass to callback function
*/
void SKYNET_set_wakeup_auth_callback(wakeupAuthCB pfxWakeupAuthCb, void *pUserData);

/** Let the socket connect to wakeup server for authentication
	@param nSocketID It got from SKYNET_socket()
	@param pszID Device ID, max. length is SKYNET_MAX_DEVICE_ID_LEN
	@param stGetWakeupInfo The information for wakup server and authentication token
	@return 0 if successful, or <0 if an error occurred and please refer to error code for detail
*/
int SKYNET_wakeup_auth(const char *pszID, st_GetWakeupInfo *pstGetWakeupInfo);
#endif


/** Invalid argument */
#define SKYNET_ERR_INVAL -1

/** Failed to get network info such as IP, netmask, MAC */
#define SKYNET_ERR_NETNOINFO -2

/** Out of memory */
#define SKYNET_ERR_NOMEM -3

/** Failed to open a new socket */
#define SKYNET_ERR_SOCKETOPEN -4

/** Invalid socket number */
#define SKYNET_ERR_BADF -5

/** The socket already in listening state */
#define SKYNET_ERR_ALREADYLISTENING -6

/** The socket already in connecting state */
#define SKYNET_ERR_ALREADYCONNECTING -7

/** Failed to create a new thread */
#define SKYNET_ERR_THREADCREATE -8

/** Failed to call TCP connect function */
#define SKYNET_ERR_TCPCONNECT -9

/** Failed to call socket send function, maybe TCP connection broken */
#define SKYNET_ERR_SOCKETSEND -10

/** Socket output queue was full, must wait a minute to try to send again */
#define SKYNET_ERR_SOCKETBUFFULL -11

/** Failed to do socket bind */
#define SKYNET_ERR_SOCKETBIND -12

/** Failed to do socket listen */
#define SKYNET_ERR_SOCKETLISTEN -13

/** Failed to set socket option */
#define SKYNET_ERR_SETSOCKETOPT -14

/** If TCP connected with ICE Server and then send register command not get response after 10 seconds */
#define SKYNET_ERR_LOGINTIMEOUT -15

/** SKYNET_listen() and SKYNET_connect() not called yet */
#define SKYNET_ERR_APIPROCESS -16

/** API operation already reached timeout */
#define SKYNET_ERR_TIMEOUT -17

/** Not set the callback function for socket receive data yet */
#define SKYNET_ERR_NOSOCKETRECVCB -18

/** Not set the callback function for socket connection yet */
#define SKYNET_ERR_NOSOCKETCONNECTCB -19

/** The socket already in listening state */
#define SKYNET_ERR_NOTLISTENING -20

/** SKYNET_alloc_buf() not called yet */
#define SKYNET_ERR_NOALLOCBUFFER -21

/** Buffer memory address not got from SKYNET_alloc_buf() */
#define SKYNET_ERR_MEMADDRESS -22

/** All ICE server TCP connected failed */
#define SKYNET_ERR_ICESERVER_NO_RESPONSE -23

/** Already do LAN discover process */
#define SKYNET_ERR_ALREADYDISCOVERING -24

/** Not start to do LAN discover process yet */
#define SKYNET_ERR_NOTSTARTDISCOVERING -25

/** Call OS mutex lock function return error */
#define SKYNET_ERR_MUTEXLOCK -26

/** Call OS mutex unlock function return error */
#define SKYNET_ERR_MUTEXUNLOCK -27

/** Call OS mutex initialize function return error */
#define SKYNET_ERR_MUTEXINIT -28

/** Remote site already close this session */
#define SKYNET_ERR_SESSIONCLOSE -29

/** Session keepalive already timeout, maybe remote site exit abnormally or network broken */
#define SKYNET_ERR_SESSIONTIMEOUT -30

/** Session exception because of TCP connection broken */
#define SKYNET_ERR_SESSIONBROKEN -31

/** Session not in a state of connected */
#define SKYNET_ERR_SESSIONNOTCONNECTED -32

/** Login status from successful to failed because reach threshold for send login failed */
#define SKYNET_ERR_LOGINBROKEN -33

/** All ICE server TCP connection is failed */
#define SKYNET_ERR_ICESERVER_CONN_FAILED -34

/** The device ID not in Seed Server database */
#define SKYNET_ERR_ID_ILLEGAL -35

/** The device ID not match any ICE Server */
#define SKYNET_ERR_ICESERVER_EMPTY -36

/** Failed to call socket recv function, maybe TCP connection broken */
#define SKYNET_ERR_SOCKETRECV -37

/** Input device ID string is in wrong format, please refer to the ID rule */
#define SKYNET_ERR_ID_FORMAT_WRONG -38

/** The user key is not match with device ID */
#define SKYNET_ERR_KEY_AUTH_FAILED -39

/** The device with the specific ID not login to ICE Server */
#define SKYNET_ERR_DEVICE_NOT_ONLINE -40

/** The ICE Server exceed max. device login capacity */
#define SKYNET_ERR_ICESERVER_MAX_LOGIN_SESSION -41

/** ICE Server already sent message to device but device not respond to ICE Server */
#define SKYNET_ERR_DEVICE_NO_RESPONSE -42

/** Network is unreachable */
#define SKYNET_ERR_NET_UNREACH -43

/** All SEED server not respond can't do TCP connect successfully */
#define SKYNET_ERR_SEEDSERVER_NO_RESPONSE -44

/** Device already reached max. session and can't be connected now */
#define SKYNET_ERR_DEVICE_MAX_SESSION -45

/** The ID duplicate using in at least two different devices */
#define SKYNET_ERR_ID_DUPLICATE -46

/** The ID is in blacklist for some reason */
#define SKYNET_ERR_ID_BLACKLIST -47

/** The data transfer occurred receive error to cause wrong protocol header */
#define SKYNET_ERR_DATA_CONFUSED -48

/** Something wrong between module and ICE Server, maybe version can't match */
#define SKYNET_ERR_ICE_PROTOCOL_EXCEPTION -49

/** The device ID already expired, please contact with vendor */
#define SKYNET_ERR_ID_EXPIRED -50

/** Failed to get socket option */
#define SKYNET_ERR_GETSOCKETOPT -51

/** The ID is in checking duplicate status, the ID can't be connected before released  */
#define SKYNET_ERR_CHECKING_ID_DUPLICATE -52

/** Connection already reach 15 seconds timeout, maybe remote site exit abnormally or network broken */
#define SKYNET_ERR_CONNECTIONTIMEOUT -53

/** Failed to open raw socket */
#define SKYNET_ERR_RAW_SOCKETOPEN -54

/** Old version module can't recognize new error reason, so module return this uniform error code */
#define SKYNET_ERR_OLD_VERSION_EXCEPTION -101

/** Device not enable wakeup mode, refer to SKYNET_listen() to see how to enable it */
#define SKYNET_ERR_NOT_ENABLE_WAKEUP_MODE -60

/** Not set the callback function for wakeup authentication yet */
#define SKYNET_ERR_NOWAKEUPAUTHCB -61

/** The socket already in wakeup authentication state */
#define SKYNET_ERR_DOINGWAKEUPAUTH -62

/** Not login in ICE Server first  */
#define SKYNET_ERR_NOTLOGINICE -63

/** All Wakeup server not respond can't do TCP connect successfully */
#define SKYNET_ERR_WAKEUPSERVER_NO_RESPONSE -64

/** Not set the callback function for query status yet */
#define SKYNET_ERR_NOQUERYSTATUSCB -65

/** The socket already in querying id status state */
#define SKYNET_ERR_ALREADYQUERYING -66

/** Not set the callback function for set id region yet */
#define SKYNET_ERR_NOSETIDREGIONCB -67

/** The socket already in setting id region state */
#define SKYNET_ERR_ALREADYSETTINGIDREGION -68

/** Without this region for setting id resion please check argument in SKYNET_set_id_region() */
#define SKYNET_ERR_WITHOUTTHISIDREGION -69

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  /* SKYNET__IOTAPI_H__  */
