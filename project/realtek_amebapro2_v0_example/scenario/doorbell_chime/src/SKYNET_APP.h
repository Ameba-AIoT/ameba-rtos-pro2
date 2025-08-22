#ifndef SKYNET__APP_H__
#define SKYNET__APP_H__

#if !defined (_WIN32)
#define __stdcall
#endif

/*Skynet Application*/
#define MAX_SEND_BUF_SIZE 256*1024
#define MAX_RECV_BUF_SIZE 2048
#define MAX_SESSION_HANDLER_NUM 3
#define MAX_CLIENT_NUMBER	3
#define MAX_SEND_SIZE 50*1024


/* a link in the queue, holds the info and point to the next Node*/
typedef struct {
	int nSocketID;
	int nCID;
	int buffer_length;
	char *buffer;
} DATA;

typedef struct Node_t {
	DATA data;
	struct Node_t *prev;
} NODE;

/* the HEAD of the Queue, hold the amount of node's that are in the queue*/
typedef struct Queue {
	NODE *head;
	NODE *tail;
	int size;
	int limit;
} Queue;


typedef struct _AV_Client {
	int destroyFlag;
	int SID; /* Socket id */
	char *pBuf;
	char *recvBuf;
	int failCnt;
	int login;
	SemaphoreHandle_t  pBuf_mutex;
	int recvRemainLen;
	int recvPos;
	int recvTotalRemainLen;
	unsigned char bEnableVideo;
	unsigned char bEnableAudio;
	unsigned char bEnableSpeaker;
	unsigned char bStopPlayBack;
	unsigned char bPausePlayBack;
	int speakerCh;
	int playBackCh;
} AV_Client;

typedef struct {
	union {
		struct {
			unsigned char  nDataSize[3];
			unsigned char  nStreamIOType; //refer to ENUM_STREAM_IO_TYPE
		} uionStreamIOHead;
		unsigned int nStreamIOHead;
	};
} st_AVStreamIOHead;

//for Video and Audio
/*
typedef struct{
	unsigned short nCodecID;	//refer to ENUM_CODECID
	unsigned char  nOnlineNum;
	unsigned char  flag;		//Video:=ENUM_VFRAME; Audio:=(ENUM_AUDIO_SAMPLERATE << 2) | (ENUM_AUDIO_DATABITS << 1) | (ENUM_AUDIO_CHANNEL)
	unsigned char  reserve[4];

	unsigned int nDataSize;
	unsigned int nTimeStamp;	//system tick
}st_AVFrameHead;
*/
typedef struct {
	unsigned short nCodecID;	//refer to ENUM_CODECID
	unsigned char  nOnlineNum;
	unsigned char  flag;		//Video:=ENUM_VFRAME; Audio:=(ENUM_AUDIO_SAMPLERATE << 2) | (ENUM_AUDIO_DATABITS << 1) | (ENUM_AUDIO_CHANNEL)
	unsigned char  top_x;
	unsigned char  top_y;
	unsigned char  width;
	unsigned char  height;

	unsigned int nDataSize;
	unsigned int nTimeStamp;	//system tick
} st_AVFrameHead;

//for IO Control
typedef struct {
	unsigned short nIOCtrlType;//refer to ENUM_IOCTRL_TYPE
	unsigned short nIOCtrlDataSize;
} st_AVIOCtrlHead;

//IOCTRL_DEV_LOGIN
typedef struct {
	unsigned int encMode ;  // 0:?? ;
	unsigned char pwd[64];
	unsigned char appid[8];      // ??嚗??孵?app???
} SLOGINReq;

typedef struct {
	unsigned short width ;
	unsigned short height ;
} SResolution ;

typedef struct {
	char ret ;        //  0:success
	// -1:wrong pwd
	// -2:wrong encMode
	// -3:wrong app

	char   fecResoult ;  // -1: nonused fec funciton ;
	// 0嚗?40x480
	// 1 : 720x480
	// 2 : 1280x720
	// 3 : 1280x960
	// 4 : 1280x1024
	// 5 : 1920x1080


	char   fecRadius ;   // -1: nonused ;

	unsigned char   changeInitPwd ; // 0:撌脫?寥?閮計wd ; 1:?芣?寥?閮計wd

	unsigned int  token;     // token

	unsigned char numCh ;   // number of channel
	SResolution res[1] ;
	unsigned char version[16]; // IPCam firmware version ex:201603151806

	unsigned char mode; // refer to ENUM_FLIP_MODE
	unsigned char  light_sw ;         // on/off switch
	unsigned char  light_min ;        // bright (0~100)
	unsigned char  night_light_sw ;         // on/off switch
	unsigned char  night_light_min ;        // bright (0~100)
	unsigned char  motion_sw ;         // on/off switch
	unsigned char  motion_value ;        // bright (0~100)
	unsigned char recType; // refer to ENUM_RECORD_TYPE
} SLOGINResp;

//IOCTRL_DEV_LOGOUT
typedef struct {
	unsigned int token ; // return from SLOGINResp
} SReq;

typedef struct {
	unsigned short cmd ;        // LAMP_CMD_GET_BRIGHT / LAMP_CMD_SET_BRIGHT
	unsigned char  sw ;         // on/off switch
	unsigned char  min ;        // bright (0~100)
} SMsgIoctrlBright   ;

typedef struct {
	unsigned char channel; // Camera Index
	unsigned char reserved[3];
} SMsgAVIoctrlAVStream;

typedef struct {
	unsigned char msg[64];
	unsigned char reserved[4];
} SMsgIoctrlSerial;

typedef struct {
	unsigned int token;           // return from SLOGINResp
	unsigned char  channel;         // channel
	unsigned char  reserved[3];
} SIoctrlGetRecordReq;

typedef struct {
	unsigned char ret ; // 0:success
	// -1:wrong token
	// -2:(TBD)

	unsigned char channel; // channel
	unsigned char recType; // refer to ENUM_RECORD_TYPE
	unsigned char reserved;
} SIoctrlGetRecordResp;

typedef struct {
	unsigned short year;	// The number of year.
	unsigned char month;	// The number of months since January, in the range 1 to 12.
	unsigned char day;		// The day of the month, in the range 1 to 31.
	unsigned char wday;		// The number of days since Sunday, in the range 0 to 6. (Sunday = 0, Monday = 1, ...)
	unsigned char hour;     // The number of hours past midnight, in the range 0 to 23.
	unsigned char minute;   // The number of minutes after the hour, in the range 0 to 59.
	unsigned char second;   // The number of seconds after the minute, in the range 0 to 59.
} STimeDay;

typedef struct {
	STimeDay stTime;
	unsigned char event;
	unsigned char status;	// 0x00: Recording file exists, Event unreaded
	// 0x01: Recording file exists, Event readed
	// 0x02: No Recording file in the event
	unsigned char reserved[2];
} SAvEvent;

/*
IOTYPE_USER_IPCAM_LISTEVENT_RESP		= 0x0319,
** @struct SMsgAVIoctrlListEventResp
*/
typedef struct {
	unsigned int  channel;		// Camera Index
	unsigned int  total;		// Total event amount in this search session
	unsigned char index;		// package index, 0,1,2...;
	// because avSendIOCtrl() send package up to 1024 bytes one time, you may want split search results to serveral package to send.
	unsigned char endflag;		// end flag; endFlag = 1 means this package is the last one.
	unsigned short count;		// how much events in this package
	SAvEvent stEvent[1];		// The first memory address of the events in this package
} SMsgAVIoctrlListEventResp;

/*
IOTYPE_USER_IPCAM_LISTEVENT_REQ			= 0x0318,
** @struct SMsgAVIoctrlListEventReq
*/
typedef struct {
	unsigned int channel; 		// Camera Index
	STimeDay stStartTime; 		// Search event from ...
	STimeDay stEndTime;	  		// ... to (search event)
	unsigned char event;  		// event type, refer to ENUM_EVENTTYPE
	unsigned char status; 		// 0x00: Recording file exists, Event unreaded
	// 0x01: Recording file exists, Event readed
	// 0x02: No Recording file in the event
	unsigned char reserved[2];
} SMsgAVIoctrlListEventReq;

/*
IOTYPE_USER_IPCAM_RECORD_PLAYCONTROL 	= 0x031A,
** @struct SMsgAVIoctrlPlayRecord
*/
typedef struct {
	unsigned int channel; // Camera Index
	unsigned int command; // play record command. refer to ENUM_PLAYCONTROL
	unsigned int seekTime;  // seek time use for Seek only
	STimeDay stTimeDay;  // Event time from ListEvent
	short seq ;
	short dummy;
} SMsgAVIoctrlPlayRecord;

/*
IOTYPE_USER_IPCAM_RECORD_PLAYCONTROL_RESP 	= 0x031B,
** @struct SMsgAVIoctrlPlayRecordResp
*/
typedef struct {
	unsigned int command;  // resp record command. refer to ENUM_PLAYCONTROL
	int result;     // >= 0 success ;  < 0 error (TBD)
	STimeDay stTimeDay;    // Event time
	unsigned int duration ; // event duration ; or event Seek time
} SMsgAVIoctrlPlayRecordResp; // only for play record start command


typedef enum {
	SIO_TYPE_UNKN,
	SIO_TYPE_VIDEO,
	SIO_TYPE_AUDIO,
	SIO_TYPE_IOCTRL,

} ENUM_STREAM_IO_TYPE;

typedef enum {
	CODECID_UNKN,
	CODECID_V_MJPEG,
	CODECID_V_MPEG4,
	CODECID_V_H264,

	CODECID_A_PCM = 0x4FF,
	CODECID_A_G711_U = 0x500,
	CODECID_A_G711_A = 0x501,
	CODECID_A_ADPCM,
	CODECID_A_SPEEX,
	CODECID_A_AMR,
	CODECID_A_AAC,
} ENUM_CODECID;

typedef enum {
	IOCTRL_TYPE_UNKN,

	IOCTRL_TYPE_VIDEO_START,
	IOCTRL_TYPE_VIDEO_STOP,
	IOCTRL_TYPE_AUDIO_START,
	IOCTRL_TYPE_AUDIO_STOP,
	IOCTRL_TYPE_SPEAKER_START,
	IOCTRL_TYPE_SPEAKER_STOP,
	IOCTRL_DEV_LOGIN						= 0x0010,
	IOCTRL_DEV_LOGOUT						= 0x0011,
	IOCTRL_DEV_SET_PWD     	 			= 0x0012,
	IOTYPE_USER_IPCAM_LISTEVENT_REQ				= 0x0318,
	IOTYPE_USER_IPCAM_LISTEVENT_RESP			= 0x0319,
	IOTYPE_USER_IPCAM_RECORD_PLAYCONTROL 		= 0x031A,
	IOTYPE_USER_IPCAM_RECORD_PLAYCONTROL_RESP 	= 0x031B,
	IOTYPE_USER_IPCAM_DEVINFO				= 0x513,
	IOTYPE_USER_IPCAM_SETPASSWORD			= 0x0331,

	IOTYPE_USER_IPCAM_LISTWIFIAP			= 0x0340,
	IOTYPE_USER_IPCAM_SETWIFI				= 0x0342,

	IOCTRL_IPC_SETRECORD					= 0x0310,
	IOCTRL_IPC_GETRECORD					= 0x0311,
	IOTYPE_USER_IPCAM_SETSTREAMCTRL		= 0x0320,
	IOTYPE_USER_IPCAM_GETSTREAMCTRL		= 0x0321,
	IOCTRL_IPC_GETFLIP						= 0x0325,
	IOCTRL_IPC_SETFLIP						= 0x0326,
	IOCTRL_IPC_GETENV						= 0x0327,
	IOCTRL_IPC_SETENV						= 0x0328,
	IOCTRL_IPC_APACER_CMD    				= 0x0400,
	IOCTRL_IPC_PTZ							= 0x0322,
	IOTYPE_USER_IPCAM_LAMP                	= 0x0515,
	IOCTRL_BD_TEST							= 0x0999,
	IOTYPE_USER_IPCAM_PTZ_COMMAND			= 0x1001,	// P2P PTZ Command Msg
	IOTYPE_USER_DEVICE_EVENT				= 0x1002,	// P2P Device Event Msg
	IOTYPE_USER_IPCAM_SCHEDULE_COMMAND		= 0x1003,	// P2P Schedule Command Msg
	IOTYPE_USER_SERIAL_TRANSPARENT			= 0x1004,	// Serial port transparent transfer Msg

} ENUM_IOCTRL_TYPE;

typedef enum {
	LAMP_CMD_UNKNOW = 0x0,
	LAMP_CMD_GET_BRIGHT = 0x01,
	LAMP_CMD_SET_BRIGHT = 0x02,
	CMD_GET_NIGHT_LIGHT_BRIGHT = 0x07,
	CMD_SET_NIGHT_LIGTH_BRIGHT = 0x08,
	CMD_GET_MOTION = 0x09,
	CMD_SET_MOTION = 0x0a,
	CMD_RESET = 0xF0,
	CMD_FORMAT_SD = 0xF1,
	CMD_GET_MOTION_VALUE = 0xF2,
	CMD_GET_LUX_VALUE = 0xF3,

} ENUM_LAMP_CMD;

typedef enum {
	VFRAME_FLAG_I	= 0x00,	// Video I Frame
	VFRAME_FLAG_P	= 0x01,	// Video P Frame
	VFRAME_FLAG_B	= 0x02,	// Video B Frame
} ENUM_VFRAME;

typedef enum {
	ASAMPLE_RATE_8K	= 0x00,
	ASAMPLE_RATE_11K = 0x01,
	ASAMPLE_RATE_12K = 0x02,
	ASAMPLE_RATE_16K = 0x03,
	ASAMPLE_RATE_22K = 0x04,
	ASAMPLE_RATE_24K = 0x05,
	ASAMPLE_RATE_32K = 0x06,
	ASAMPLE_RATE_44K = 0x07,
	ASAMPLE_RATE_48K = 0x08,
} ENUM_AUDIO_SAMPLERATE;

typedef enum {
	ADATABITS_8		= 0,
	ADATABITS_16	= 1,
} ENUM_AUDIO_DATABITS;

typedef enum {
	ACHANNEL_MONO	= 0,
	ACHANNEL_STERO	= 1,
} ENUM_AUDIO_CHANNEL;

// AVIOCTRL Event Type
typedef enum {
	AVIOCTRL_EVENT_ALL					= 0x00,	// all event type(general APP-->IPCamera)
	AVIOCTRL_EVENT_MOTIONDECT			= 0x01,	// motion detect start//==s==
	AVIOCTRL_EVENT_VIDEOLOST			= 0x02,	// video lost alarm
	AVIOCTRL_EVENT_IOALARM				= 0x03, // io alarmin start //---s--

	AVIOCTRL_EVENT_MOTIONPASS			= 0x04, // motion detect end  //==e==
	AVIOCTRL_EVENT_VIDEORESUME			= 0x05,	// video resume
	AVIOCTRL_EVENT_IOALARMPASS			= 0x06, // IO alarmin end   //---e--

	AVIOCTRL_EVENT_EXPT_REBOOT			= 0x10, // system exception reboot
	AVIOCTRL_EVENT_SDFAULT				= 0x11, // sd record exception
} ENUM_EVENTTYPE;

// AVIOCTRL Play Record Command
typedef enum {
	AVIOCTRL_RECORD_PLAY_PAUSE			= 0x00,
	AVIOCTRL_RECORD_PLAY_STOP			= 0x01,
	AVIOCTRL_RECORD_PLAY_STEPFORWARD	= 0x02, //now, APP no use
	AVIOCTRL_RECORD_PLAY_STEPBACKWARD	= 0x03, //now, APP no use
	AVIOCTRL_RECORD_PLAY_FORWARD		= 0x04, //now, APP no use
	AVIOCTRL_RECORD_PLAY_BACKWARD		= 0x05, //now, APP no use
	AVIOCTRL_RECORD_PLAY_SEEKTIME		= 0x06, //now, APP no use
	AVIOCTRL_RECORD_PLAY_END			= 0x07,
	AVIOCTRL_RECORD_PLAY_START			= 0x10,
} ENUM_PLAYCONTROL;

void skynet_get_sntp_time();


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  /* SKYNET__APP_H__  */
