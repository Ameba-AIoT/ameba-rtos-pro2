/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "MQTTClient.h"
#include "wifi_conf.h"
#include "lwip_netconf.h"
#include "log_service.h"

#ifdef USE_ATCMD_MQTT
#include "atcmd_mqtt.h"
#endif

#define MQTT_SELECT_TIMEOUT 1

typedef struct {
	char clientID[64];
	char username[64];
	char password[64];
	char address[64];
	char pub_topic[128];
	char sub_topic[128];
	uint16_t port;
	uint8_t configured;  // Flag to check if all parameters are set
} mqtt_config_t;

static MQTTClient client;
static mqtt_config_t mqtt_config = {
	.clientID = "",
	.username = "",
	.password = "",
	.address = "",
	.pub_topic = "",
	.sub_topic = "",
	.port = 1883,
	.configured = 0
};

static void atcmd_userctrl_init(void);
void MQTTPublishMessage(MQTTClient *c, char *);

// Check if all required MQTT parameters are configured
static uint8_t check_mqtt_config_complete(void)
{
	if (strlen(mqtt_config.clientID) > 0 &&
		strlen(mqtt_config.username) > 0 &&
		strlen(mqtt_config.password) > 0 &&
		strlen(mqtt_config.address) > 0 &&
		strlen(mqtt_config.pub_topic) > 0 &&
		strlen(mqtt_config.sub_topic) > 0) {
		return 1;
	}
	return 0;
}

static void messageArrived(MessageData *data)
{
	char payload_str[256];
	int len = data->message->payloadlen;

	if (len > sizeof(payload_str) - 1) {
		len = sizeof(payload_str) - 1;
	}

	memcpy(payload_str, data->message->payload, len);
	payload_str[len] = '\0';
	mqtt_printf(MQTT_INFO, "Message arrived on topic %.*s: %.*s\n", data->topicName->lenstring.len, data->topicName->lenstring.data, data->message->payloadlen,
				(char *)data->message->payload);

#ifdef USE_ATCMD_MQTT
	
	// use atcmd_mqtt to handle
	char response[4096];
	int resp_len = atcmd_mqtt_process(payload_str, len,
										response, sizeof(response));

	if (resp_len > 0) {
		MQTTMessage msg;
		msg.qos = QOS1;
		msg.retained = 0;
		msg.payload = response;
		msg.payloadlen = resp_len;

		// check socket
		if (client.ipstack != NULL) {
			mqtt_printf(MQTT_INFO, "Socket fd: %d\n", client.ipstack->my_socket);
		}

		int rc = MQTTPublish(&client, mqtt_config.pub_topic, &msg);
		mqtt_printf(MQTT_INFO, "MQTTPublish to %s returned: %d\n", mqtt_config.pub_topic, rc);
	} else {
		// If no response, send default message
		char no_response[] = "ERROR: No response\r\n";
		MQTTMessage msg;
		msg.qos = QOS1;
		msg.retained = 0;
		msg.payload = no_response;
		msg.payloadlen = strlen(no_response);

		MQTTPublish(&client, mqtt_config.pub_topic, &msg);
	}
	
#endif
}
//This example is original and cannot restart if failed. To use this example, define WAIT_FOR_ACK and not define MQTT_TASK in MQTTClient.h
void prvMQTTEchoTask(void *pvParameters)
{
	/* To avoid gcc warnings */
	(void) pvParameters;

	/* connect to broker.emqx.io, subscribe to a topic, send and receive messages regularly every 5 sec */
	MQTTClient client;
	Network network;
	unsigned char sendbuf[512], readbuf[80];
	int rc = 0, count = 0;
	MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;
	const char *address = mqtt_config.address;
	const char *sub_topic = mqtt_config.sub_topic;

	//const char *pub_topic = "ha/test/pro2";

	memset(readbuf, 0x00, sizeof(readbuf));

	NetworkInit(&network);
	MQTTClientInit(&client, &network, 30000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

	mqtt_printf(MQTT_INFO, "Wait Wi-Fi to be connected.");
	while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
	mqtt_printf(MQTT_INFO, "Wi-Fi connected.");

	mqtt_printf(MQTT_INFO, "Connect Network \"%s\"", address);
	while ((rc = NetworkConnect(&network, (char *)address, 1883)) != 0) {
		mqtt_printf(MQTT_INFO, "Return code from network connect is %d\n", rc);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	mqtt_printf(MQTT_INFO, "\"%s\" Connected", address);

	connectData.MQTTVersion = 3;
	connectData.clientID.cstring = mqtt_config.clientID;
	connectData.username.cstring = mqtt_config.username;
	connectData.password.cstring = mqtt_config.password;

	mqtt_printf(MQTT_INFO, "Start MQTT connection");
	while ((rc = MQTTConnect(&client, &connectData)) != 0) {
		mqtt_printf(MQTT_INFO, "Return code from MQTT connect is %d\n", rc);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	mqtt_printf(MQTT_INFO, "MQTT Connected");

	mqtt_printf(MQTT_INFO, "Subscribe to Topic: %s", sub_topic);
	if ((rc = MQTTSubscribe(&client, sub_topic, QOS2, messageArrived)) != 0) {
		mqtt_printf(MQTT_INFO, "Return code from MQTT subscribe is %d\n", rc);
	}

	mqtt_printf(MQTT_INFO, "Publish Topics: %s", mqtt_config.pub_topic);
	while (1) {
		MQTTMessage message;
		char payload[300];

		if (++count == 0) {
			count = 1;
		}

		message.qos = QOS1;
		message.retained = 0;
		message.payload = payload;
		sprintf(payload, "hello from AMEBA %d", count);
		message.payloadlen = strlen(payload);

		if ((rc = MQTTPublish(&client, mqtt_config.pub_topic, &message)) != 0) {
			mqtt_printf(MQTT_INFO, "Return code from MQTT publish is %d\n", rc);
		}
		if ((rc = MQTTYield(&client, 1000)) != 0) {
			mqtt_printf(MQTT_INFO, "Return code from yield is %d\n", rc);
		}
		vTaskDelay(5000);
	}
	/* do not return */
}

#if defined(MQTT_TASK)
void MQTTPublishMessage(MQTTClient *c, char *topic)
{
	int rc = 0;
	static int count = 0;
	MQTTMessage message;
	char payload[300];
	message.qos = QOS1;
	message.retained = 0;
	message.payload = payload;

	if (c->mqttstatus == MQTT_RUNNING) {
		count++;
		sprintf(payload, "hello from AMEBA %d", count);
		message.payloadlen = strlen(payload);
		mqtt_printf(MQTT_INFO, "Publish Topic %s : %d", topic, count);
		if ((rc = MQTTPublish(c, topic, &message)) != 0) {
			mqtt_printf(MQTT_INFO, "Return code from MQTT publish is %d\n", rc);
			MQTTSetStatus(c, MQTT_START);
			c->ipstack->disconnect(c->ipstack);
		}
	}

}


static void prvMQTTTask(void *pvParameters)
{
	/* To avoid gcc warnings */
	(void) pvParameters;
	Network network;
	static unsigned char sendbuf[MQTT_SENDBUF_LEN], readbuf[MQTT_READBUF_LEN];
	int rc = 0, mqtt_pub_count = 0;
	MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

	if (check_mqtt_config_complete()) {
		mqtt_config.configured = 1;
		mqtt_printf(MQTT_INFO, "Using pre-configured settings\n");
	} else {
		mqtt_printf(MQTT_INFO, "Waiting for configuration via AT commands...\n");
	}

	// Wait for configuration to be complete
	while (!mqtt_config.configured) {
		if (check_mqtt_config_complete()) {
			mqtt_config.configured = 1;
			mqtt_printf(MQTT_INFO, "Configuration detected\n");
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	// please change the username, password, and address to your corresponding information
	connectData.MQTTVersion = 3;
	connectData.clientID.cstring = mqtt_config.clientID;
	connectData.username.cstring = mqtt_config.username;
	connectData.password.cstring = mqtt_config.password;
	const char *address = mqtt_config.address;
	char *sub_topic[1];
	sub_topic[0] = mqtt_config.sub_topic;

	NetworkInit(&network);
	MQTTClientInit(&client, &network, 30000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

	while (1) {
		while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
			mqtt_printf(MQTT_INFO, "Wait Wi-Fi to be connected.");
			vTaskDelay(5000 / portTICK_PERIOD_MS);
		}

		fd_set read_fds;
		fd_set except_fds;
		struct timeval timeout;

		FD_ZERO(&read_fds);
		FD_ZERO(&except_fds);
		timeout.tv_sec = MQTT_SELECT_TIMEOUT;
		timeout.tv_usec = 0;

		if (network.my_socket >= 0) {
			FD_SET(network.my_socket, &read_fds);
			FD_SET(network.my_socket, &except_fds);
			rc = FreeRTOS_Select(network.my_socket + 1, &read_fds, NULL, &except_fds, &timeout);
			if (FD_ISSET(network.my_socket, &except_fds)) {
				mqtt_printf(MQTT_INFO, "except_fds is set");
				MQTTSetStatus(&client, MQTT_START); //my_socket will be close and reopen in MQTTDataHandle if STATUS set to MQTT_START
			} 
		}

#ifdef USE_ATCMD_MQTT
		// Initialized after the first successful connection
		static int atcmd_inited = 0;
		if (client.isconnected && !atcmd_inited) {
			atcmd_mqtt_init();
			atcmd_mqtt_set_client(&client, mqtt_config.pub_topic);
			atcmd_inited = 1;
		}
#endif

		MQTTDataHandle(&client, &read_fds, &connectData, messageArrived, (char *)address, sub_topic, 1);

	}
}
#endif

void vStartMQTTTasks(uint16_t usTaskStackSize, UBaseType_t uxTaskPriority)
{
	BaseType_t x = 0L;
	printf("\nExample: Mqtt \n");
#if defined(MQTT_TASK)
	xTaskCreate(prvMQTTTask,	/* The function that implements the task. */
				"MQTTTask",			/* Just a text name for the task to aid debugging. */
				usTaskStackSize,	/* The stack size is defined in FreeRTOSIPConfig.h. */
				(void *)x,		/* The task parameter, not used in this case. */
				uxTaskPriority,		/* The priority assigned to the task is defined in FreeRTOSConfig.h. */
				NULL);				/* The task handle is not used. */
#else
	xTaskCreate(prvMQTTEchoTask,	/* The function that implements the task. */
				"MQTTEcho0",			/* Just a text name for the task to aid debugging. */
				usTaskStackSize + 128,	/* The stack size is defined in FreeRTOSIPConfig.h. */
				(void *)x,		/* The task parameter, not used in this case. */
				uxTaskPriority,		/* The priority assigned to the task is defined in FreeRTOSConfig.h. */
				NULL);				/* The task handle is not used. */
#endif

}

void example_mqtt(void)
{
	atcmd_userctrl_init();
	vStartMQTTTasks(4096, tskIDLE_PRIORITY + 4);
}

static void fMQTT_CLIENT(void *arg)
{
	strncpy(mqtt_config.clientID, (char *)arg, sizeof(mqtt_config.clientID) - 1);
	mqtt_config.clientID[sizeof(mqtt_config.clientID) - 1] = '\0';
	mqtt_printf(MQTT_INFO, "Client ID set to: %s\n", mqtt_config.clientID);
}

static void fMQTT_USER(void *arg)
{
	strncpy(mqtt_config.username, (char *)arg, sizeof(mqtt_config.username) - 1);
	mqtt_config.username[sizeof(mqtt_config.username) - 1] = '\0';
	mqtt_printf(MQTT_INFO, "Username set to: %s\n", mqtt_config.username);
}

static void fMQTT_PASS(void *arg)
{
	strncpy(mqtt_config.password, (char *)arg, sizeof(mqtt_config.password) - 1);
	mqtt_config.password[sizeof(mqtt_config.password) - 1] = '\0';
	mqtt_printf(MQTT_INFO, "Password set\n");
}

static void fMQTT_ADDR(void *arg)
{
	strncpy(mqtt_config.address, (char *)arg, sizeof(mqtt_config.address) - 1);
	mqtt_config.address[sizeof(mqtt_config.address) - 1] = '\0';

	client.mqttstatus = MQTT_START;
	mqtt_printf(MQTT_INFO, "Broker address set to: %s\n", mqtt_config.address);
}

static void fMQTT_PORT(void *arg)
{
	mqtt_config.port = atoi((char *)arg);
	mqtt_printf(MQTT_INFO, "Port set to: %d\n", mqtt_config.port);
}

static void fMQTT_PUB(void *arg)
{
	strncpy(mqtt_config.pub_topic, (char *)arg, sizeof(mqtt_config.pub_topic) - 1);
	mqtt_config.pub_topic[sizeof(mqtt_config.pub_topic) - 1] = '\0';
	mqtt_printf(MQTT_INFO, "Publish topic set to: %s\n", mqtt_config.pub_topic);
}

static void fMQTT_SUB(void *arg)
{
	strncpy(mqtt_config.sub_topic, (char *)arg, sizeof(mqtt_config.sub_topic) - 1);
	mqtt_config.sub_topic[sizeof(mqtt_config.sub_topic) - 1] = '\0';

	client.mqttstatus = MQTT_START;
	mqtt_printf(MQTT_INFO, "Subscribe topic set to: %s\n", mqtt_config.sub_topic);
}

static void fMQTT_START(void *arg)
{
	if (check_mqtt_config_complete()) {
		mqtt_config.configured = 1;
		mqtt_printf(MQTT_INFO, "Configuration complete. MQTT connection will start.\n");
	} else {
		mqtt_printf(MQTT_INFO, "Configuration incomplete. Please set all required parameters:\n");
		mqtt_printf(MQTT_INFO, "- Client ID: %s\n", strlen(mqtt_config.clientID) > 0 ? "Set" : "Not set");
		mqtt_printf(MQTT_INFO, "- Address: %s\n", strlen(mqtt_config.address) > 0 ? "Set" : "Not set");
		mqtt_printf(MQTT_INFO, "- Username: %s\n", strlen(mqtt_config.username) > 0 ? "Set" : "Not set");
		mqtt_printf(MQTT_INFO, "- Password: %s\n", strlen(mqtt_config.password) > 0 ? "Set" : "Not set");
		mqtt_printf(MQTT_INFO, "- Pub Topic: %s\n", strlen(mqtt_config.pub_topic) > 0 ? "Set" : "Not set");
		mqtt_printf(MQTT_INFO, "- Sub Topic: %s\n", strlen(mqtt_config.sub_topic) > 0 ? "Set" : "Not set");
	}
}

static void fMQTT_STATUS(void *arg)
{
	mqtt_printf(MQTT_INFO, "\n[MQTT Configuration Status]\n");
	mqtt_printf(MQTT_INFO, "Client ID: %s\n", mqtt_config.clientID);
	mqtt_printf(MQTT_INFO, "Username: %s\n", mqtt_config.username);
	mqtt_printf(MQTT_INFO, "- Password: %s\n", strlen(mqtt_config.password) > 0 ? "Set" : "Not set");
	mqtt_printf(MQTT_INFO, "Address: %s\n", mqtt_config.address);
	mqtt_printf(MQTT_INFO, "Port: %d\n", mqtt_config.port);
	mqtt_printf(MQTT_INFO, "Publish Topic: %s\n", mqtt_config.pub_topic);
	mqtt_printf(MQTT_INFO, "Subscribe Topic: %s\n", mqtt_config.sub_topic);
	mqtt_printf(MQTT_INFO, "Configured: %s\n", mqtt_config.configured ? "Yes" : "No");
	mqtt_printf(MQTT_INFO, "MQTT Status: %s\n", client.isconnected ? "Connected" : "Disconnected");
}

static log_item_t userctrl_items[] = {
	{"MQTTCLIENT", fMQTT_CLIENT,},    // MQTTCLIENT=<clientID>
	{"MQTTUSER", fMQTT_USER,},        // MQTTUSER=<username>
	{"MQTTPASS", fMQTT_PASS,},        // MQTTPASS=<password>
	{"MQTTADDR", fMQTT_ADDR,},        // MQTTADDR=<broker_address>
	{"MQTTPORT", fMQTT_PORT,},        // MQTTPORT=<port>
	{"MQTTPUB", fMQTT_PUB,},          // MQTTPUB=<publish_topic>
	{"MQTTSUB", fMQTT_SUB,},          // MQTTSUB=<subscribe_topic>
	{"MQTTSTART", fMQTT_START,},      // MQTTSTART
	{"MQTTSTATUS", fMQTT_STATUS,},    // MQTTSTATUS
};

static void atcmd_userctrl_init(void)
{
	log_service_add_table(userctrl_items, sizeof(userctrl_items) / sizeof(userctrl_items[0]));
}
/*-----------------------------------------------------------*/


