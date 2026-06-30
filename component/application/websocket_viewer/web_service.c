#include "FreeRTOS.h"
#include <freertos_service.h>
#include <osdep_service.h>
#include "task.h"
#include <message_buffer.h>
#include "diag.h"
#include "hal.h"
#include "log_service.h"
#include <platform_opts.h>
#include <pthread.h>
#include <vfs.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <cJSON.h>
#include "wifi_constants.h"
#include "wifi_conf.h"

#include <wsfs_server.h>

#define MAX_CLIENTS 4

enum {
    AI_VIDEO_CHN_1 = 0,
    AI_VIDEO_CHN_NUM
};

typedef void (*process_cmd_t)(wsfs_connection_t *conn, void *p);

typedef struct app_cmd_t_ {
    const char *name;
    process_cmd_t func;
} app_cmd_t;

// user data here
enum {
    STATE_IDLE = 0,
    STATE_CLOSING,
    STATE_STREAMING,
};

enum {
    STREAM_H264_CH1 = 1 << 0,
    STREAM_H264_CH2 = 1 << 1,
    STREAM_PCM  = 1 << 2
};

typedef struct chunk_t_ {
    uint8_t *start;
    size_t size;
} chunk_t;

typedef struct app_t_ {
    int state;
    int stream_mode;
    pthread_t thread;
    wsfs_connection_t *conn; // bidirectional link
} app_t;
static app_t apps[MAX_CLIENTS] = {0};

#define STAT_PERIOD_SECONDS 60

// Structure to hold frame statistics
typedef struct {
    uint32_t sent_count;
    uint32_t lost_count;
    TickType_t start_tick;
} FrameStats;

// Initialize statistics
static FrameStats frame_stats = {0, 0, 0};

//The two bytes before the FRAME, 0x00, 0xff represent audio, 0x00, 0xfe represents channel 1, and 0x00, 0xfd represents channel 2
#define FRAME_HEADER_SIZE (2)

static inline int check_stream_id(int stream_id) {
    if (stream_id >= AI_VIDEO_CHN_1 && stream_id < AI_VIDEO_CHN_NUM) {
        return 0;
    }
    return -1;
}

static size_t check_size(uint8_t *start, size_t size)
{
    for (int i=0; i<(int)size; i++) {
        if ((start[i] == 0x01) && (start[i-1] == 0x00) && (start[i-2] == 0x00)) {
            if (((i-3)>=0) && (start[i-3] == 0)) {
                return i-3;
            } else {
                return i-2;
            }
        }
    }

    return size;
}

static void insert_chn_header(chunk_t *pc, int stream_id)
{
    if (check_stream_id(stream_id) < 0) {
        printf("%d illegal stream id: %d\r\n", __LINE__, stream_id);
        return;
    }

    uint8_t *p = pc->start;
    if (stream_id == AI_VIDEO_CHN_1) {
        *(--p) = 0xFE;
        *(--p) = 0x00;
    } else {
        *(--p) = 0xFD;
        *(--p) = 0x00;
    }

    pc->start = p;
    pc->size += FRAME_HEADER_SIZE;
}

static void next_chunk(int stream_id, uint8_t *start, size_t size, chunk_t *pc)
{
    memset(pc, 0, sizeof(chunk_t));
    for (int i=2; i<(int)size; i++) {
        if ((start[i] == 0x01) && (start[i-1] == 0x00) && (start[i-2] == 0x00)) {
            i++;
            pc->start = start + i;
            pc->size = check_size(pc->start, size-i);
            insert_chn_header(pc, stream_id);
            return;
        }
    }
}

static int ws_vstream_enabled(app_t *app, int stream_id)
{
    if (app != NULL && app->conn != NULL) {
        // Check if the connection is still valid
        if (app->state == STATE_STREAMING) {
            if (app->stream_mode & STREAM_H264_CH1 && stream_id == AI_VIDEO_CHN_1)
                return 1;
        }
        return 0;
    } else {
        return 0;
    }

    return 0;
}

enum {
    NALU_TYPE_SLICE = 1,
    NALU_TYPE_DPA,
    NALU_TYPE_DPB,
    NALU_TYPE_DPC,
    NALU_TYPE_IDR,  // i frame
    NALU_TYPE_SEI,
    NALU_TYPE_SPS,
    NALU_TYPE_PPS,
    NALU_TYPE_AUD,
    NALU_TYPE_EOSEQ,
    NALU_TYPE_EOSTREAM,
    NALU_TYPE_FILL
};

#define BASELINE_PROFILE		66
#define MAIN_PROFILE			77
#define EXTENDED_PROFILE		88
#define HIGH_PROFILE			100
#define HIGH_10_PROFILE			110
#define HIGH_422_PROFILE		122
#define HIGH_444_PROFILE		144

static uint8_t *bitstream = NULL;
static const size_t bs_size = 320*1024;

int wsh264_recv_frame(int stream_id, uint8_t *data_addr, const uint32_t size);
void *av_streaming(void* data)
{
    (void)data;

    app_t *apps = (app_t *)data;
    uint8_t *start = bitstream;
    int size = bs_size;
    do {
        
        int h264_size = 0;
        for (int stream_id = 0; stream_id < AI_VIDEO_CHN_NUM; stream_id++) {
            chunk_t chunk;
            h264_size = wsh264_recv_frame(stream_id, bitstream, bs_size);

            if (h264_size > 0) {
                start = bitstream;
                size = h264_size;
                //printf("H264 size: %d\r\n", h264_size);
                //printf("[%d] %02X %02X %02X %02X %02X %02X %02X %02X\r\n", stream_id, bitstream[0], bitstream[1],bitstream[2],bitstream[3],
                // 																	  bitstream[4], bitstream[5],bitstream[6],bitstream[7]);
                for (;;) {
                    next_chunk(stream_id, start, size, &chunk);
                    size -= ((chunk.start - start) + chunk.size);
                    start = chunk.start + chunk.size;
                    for (int i = 0; i < MAX_CLIENTS; i++) {
                        if (ws_vstream_enabled(&apps[i], stream_id) && chunk.size > 0) {
                            //printf("%d %02X %02X %02X %02X %u\r\n", i, chunk.start[0], chunk.start[1],chunk.start[2],chunk.start[3],(unsigned)chunk.size);
                            
                            wsfs_status_t status = wsfs_connection_send_binary(apps[i].conn, chunk.start, chunk.size);
                            if (status == WSFS_STATUS_OK) {
                                //printf("%02X %02X %02X %02X %u\r\n", chunk.start[0], chunk.start[1],chunk.start[2],chunk.start[3],(unsigned)chunk.size);
                            } else {
                                printf("Send failed: status=%d, size=%u\r\n", (int)status, (unsigned)chunk.size);
                                //It may be necessary to disconnect or mark the client as unavailable.
                                if (status == WSFS_STATUS_INVALID_STATE) {
                                    printf("Client %d connection is invalid, marking as disconnected\r\n", i);
                                    apps[i].conn = NULL;
                                    apps[i].state = STATE_IDLE;
                                    apps[i].stream_mode = 0;
                                }
                            }
                        }
                    }
                    if (chunk.start == 0) 
                        break;
                }
                //printf("--------------------------------------------------------------------\r\n");
            }
        }
        
        

        if (h264_size == 0) {
            vTaskDelay(5);
            continue;
        }

    } while (1);
}

static void start_stream(wsfs_connection_t *conn, void *p)
{
    printf("start_stream called\n");
    app_t *app = (app_t *)wsfs_connection_get_user_data(conn);
    if (app) {
        app->stream_mode |= STREAM_H264_CH1;
        app->state = STATE_STREAMING;  // Set the status to streaming
    }
}


static void stop_stream(wsfs_connection_t *conn, void *p)
{
    printf("stop_stream called\n");
    app_t *app = (app_t *)wsfs_connection_get_user_data(conn);
    if (app) {
        app->stream_mode &= ~STREAM_H264_CH1;
        app->state = STATE_IDLE;  // Set the status to idle
    }
}


static void start_model(wsfs_connection_t *conn, void *p)
{
    printf("%s called\r\n", __FUNCTION__);
}

static void stop_model(wsfs_connection_t *conn, void *p)
{
    printf("%s called\r\n", __FUNCTION__);
}

static void get_model_status(wsfs_connection_t *conn, void *p)
{
    printf("%s called\r\n", __FUNCTION__);
}

static void get_version(wsfs_connection_t *conn, void *p)
{
    printf("%s called\r\n", __FUNCTION__);
}


static app_cmd_t app_cmd[] = {
    {"start_stream", start_stream},
    {"stop_stream", stop_stream},
    {"start_model", start_model},
    {"stop_model", stop_model},
    {"get_model_status", get_model_status},
    {"get_version", get_version},
    {0, 0}
};

/**
 * @brief Called when a client connects to the server.
 *
 * @param conn Client connection. The @p conn parameter is used
 * in order to send messages and retrieve informations about the
 * client.
 */
static void on_open(wsfs_connection_t *conn)
{
	printf("[wsfs_echo] connection opened from %s:%u\n",
		   wsfs_connection_peer_ip(conn), 
		   wsfs_connection_peer_port(conn));
	
	// Find available slot for new client
	int i = 0;
	for (i = 0; i < MAX_CLIENTS; i++) {
		if (apps[i].conn == NULL) {
			apps[i].state = STATE_IDLE;
			apps[i].stream_mode = 0;
			apps[i].conn = conn;
			wsfs_connection_set_user_data(conn, &apps[i]);
			printf("Client assigned to slot %d\n", i);
			break;
		}
	}

	if (i == MAX_CLIENTS) {
		printf("Too many clients, closing connection.\r\n");
		wsfs_connection_close(conn, 1013); // Try again later
	}
}



/**
 * @brief Called when a client disconnects from the server.
 *
 * @param conn Client connection.
 * @param close_code WebSocket close code.
 */
static void on_close(wsfs_connection_t *conn, uint16_t close_code)
{
	printf("[wsfs_echo] connection closed from %s:%u (code=%u)\n",
		   wsfs_connection_peer_ip(conn), 
		   wsfs_connection_peer_port(conn),
		   (unsigned)close_code);
	
	app_t *app = (app_t *)wsfs_connection_get_user_data(conn);
	if (app) {
		// Stop streaming for this client
		app->stream_mode = 0;
		app->conn = NULL;
		app->state = STATE_IDLE;
		printf("Client slot freed\n");
	}
}




static void process_command(wsfs_connection_t *client, const char *msg)
{
    app_cmd_t *cmdtbl = (app_cmd_t *)app_cmd;
    cJSON *root = cJSON_Parse(msg);
    if (root == NULL) return;

    //for 2023 computex demo:control audio and light
    if (cJSON_GetObjectItem(root, "type") != NULL) {
        cJSON_Delete(root);
        return;
    }

    if (cJSON_GetObjectItem(root, "cmd") == NULL) {
        cJSON_Delete(root);
        return;
    }

    char *cmd = cJSON_GetObjectItem(root, "cmd")->valuestring;
    if (cmd) {
        printf("Received %s command\r\n", cmd);
    }
    cJSON *param = cJSON_GetObjectItem(root, "param");
    for (;;) {
        if (cmdtbl->name == NULL || cmd == NULL)
            break;
        
        if (strcmp(cmdtbl->name, cmd)==0) {
            printf("Process %s command\r\n", cmd);
            if (strcmp("set_config", cmd)==0)
                cmdtbl->func(client, (void*)msg);
            else
                cmdtbl->func(client, param);
            break;
        }
        cmdtbl++;
    }
    cJSON_Delete(root);
}

/**
 * @brief Called when a message is received from a client.
 *
 * @param conn Client connection.
 * @param msg Received message, this message can be a text or binary message.
 */
static void on_message(wsfs_connection_t *conn,
	       const wsfs_message_view_t *msg)
{
	printf("[wsfs_echo] received message (%zu bytes, opcode=%u)\n",
	       msg != NULL ? msg->length : 0,
	       msg != NULL ? (unsigned)msg->opcode : 0);
	if (msg == NULL)
		return;

	if (msg->opcode == WSFS_OPCODE_TEXT) {
		// Process JSON command
		char *text_msg = malloc(msg->length + 1);
		if (text_msg != NULL) {
			memcpy(text_msg, msg->data, msg->length);
			text_msg[msg->length] = '\0';
			printf("Processing command: %s\n", text_msg);
			process_command(conn, text_msg);
			free(text_msg);
		}
	} else if (msg->opcode == WSFS_OPCODE_BINARY) {
		// Handle binary data if needed
		printf("Received binary data\n");
	}
}



static void websocket_thread(void *param)
{
    (void)param;
    
    wsfs_server_config_t config;
	wsfs_server_config_defaults(&config);
	config.callbacks.on_open = on_open;
	config.callbacks.on_message = on_message;
	config.callbacks.on_close = on_close;
    config.port = 8081;
    config.max_clients = MAX_CLIENTS;

	wsfs_server_t server = {0};
	wsfs_status_t status = wsfs_server_init(&server, &config);
	if (status != WSFS_STATUS_OK) {
		printf("Failed to initialize WebSocket server (status=%d)\n",
			(int)status);
		vTaskDelete(NULL);
		return;
	}

	printf("WebSocket server starting on port %u\n", config.port);
	status = wsfs_server_run(&server);
	if (status != WSFS_STATUS_OK) {
		printf("wsfs_server_run returned status=%d\n",
			(int)status);
	}

    printf("WebSocket server exited\r\n");
    wsfs_server_deinit(&server);
    vTaskDelete(NULL);
}

//MessageBufferHandle_t h264MessageBuffer = NULL;
const uint32_t h264MessageBufferSizeBytes = bs_size*6;

typedef struct h264_queue_s {
    MessageBufferHandle_t mbuffer;
} h264_queue_t;

h264_queue_t h264_queue[2] = {0};

int init_h264_msgbuf(void)
{
    bitstream = (uint8_t *)malloc(bs_size);

    for (int i=0; i<AI_VIDEO_CHN_NUM; i++) {
        h264_queue[i].mbuffer = xMessageBufferCreate( h264MessageBufferSizeBytes );
        if( h264_queue[i].mbuffer == NULL || bitstream == NULL) {
            printf("Failed to create message buffer.\r\n");
            while(1);
        }
    }

    return 0;
}

// This function should be called each time a frame is received
void websocket_viewer_frame_static(int status)
{
    TickType_t now_tick = xTaskGetTickCount();

    // Initialize start tick if this is the first call
    if (frame_stats.start_tick == 0) {
        frame_stats.start_tick = now_tick;
    }

    // Accumulate statistics
    if (status == 0) {
        frame_stats.sent_count++;   // Assume 0 = success
    } else {
        frame_stats.lost_count++;   // Other = failure / lost
    }

    // Check if STAT_PERIOD_SECONDS have passed
    if ((now_tick - frame_stats.start_tick) >= (STAT_PERIOD_SECONDS * configTICK_RATE_HZ)) {
        // Print the statistics
        printf("Last %d seconds: Sent=%u, Lost=%u\n", 
               STAT_PERIOD_SECONDS, frame_stats.sent_count, frame_stats.lost_count);

        // Reset statistics
        frame_stats.sent_count = 0;
        frame_stats.lost_count = 0;

        // Reset start tick
        frame_stats.start_tick = now_tick;
    }
}




int wsh264_send_frame(int stream_id, uint8_t *data_addr, uint32_t size)
{
    const TickType_t x5ms = pdMS_TO_TICKS( 5 );
	int status = 0;
    if (check_stream_id(stream_id) < 0) {
        printf("%d illegal stream id: %d\r\n", __LINE__, stream_id);
        return -1;
    }

    if (h264_queue[stream_id].mbuffer == NULL) return;

    if (size > bs_size) {
        printf("h264 frame size is too large: %lu bytes!!!\r\n", size);
        return -1;
    }

    uint32_t xBytesSent = xMessageBufferSend( h264_queue[stream_id].mbuffer, ( void * ) data_addr, size, x5ms );
    if( xBytesSent != size )
    {
        //printf("Failed to send data to message buffer. (h264)\r\n");
		//return -1;
		status = -1;
    }
	websocket_viewer_frame_static(status);
	return 0;
}

int wsh264_recv_frame(int stream_id, uint8_t *data_addr, const uint32_t size)
{
    if (check_stream_id(stream_id) < 0) {
        printf("%d illegal stream id: %d\r\n", __LINE__, stream_id);
        return -1;
    }

    if (h264_queue[stream_id].mbuffer == NULL) return -1;

    uint32_t xReceivedBytes = xMessageBufferReceive(h264_queue[stream_id].mbuffer, ( void * ) data_addr, size, 0 );
    return xReceivedBytes;
}

void start_httpd(void);
static void websocket_viewer_thread(void *param)
{
    int ret = 0;
    
    wifi_set_powersave_mode(IPS_MODE_NONE, LPS_MODE_NONE);

    //create av streaming thread
    pthread_t thread;
    pthread_attr_t attr;
    size_t stacksize;
    pthread_attr_init(&attr);
    pthread_attr_getstacksize(&attr, &stacksize);
    stacksize *= 16;
    pthread_attr_setstacksize(&attr, stacksize);
    pthread_create(&thread, NULL, av_streaming, apps);

    // Initializing the HTTP server too early may cause the browser to crash when connecting, 
	// as the video has not finished initializing yet. 
	// Therefore, we wait for the video to complete initialization before initializing the HTTP server.

    vTaskDelay(1000);
    start_httpd();
    if (xTaskCreate(websocket_thread, ((const char *)"websocket_thread"), 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("\n\r%s xTaskCreate(websocket_thread) failed", __FUNCTION__);
        vTaskDelete(NULL);
    }

init_error:
    vTaskDelete(NULL);
}

void start_websocket_viewer(void)
{
    init_h264_msgbuf();

    if (xTaskCreate(websocket_viewer_thread, ((const char *)"websocket_viewer_thread"), 2048*16, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
         printf("\n\r%s xTaskCreate(websocket_viewer_thread) failed", __FUNCTION__);
    }
}
