/**
  ******************************************************************************
  * @file           : main.c
  * @author         : Mauricio Barroso Benavides
  * @date           : Feb 9, 2022
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2022 Mauricio Barroso Benavides
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to
  * deal in the Software without restriction, including without limitation the
  * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
  * sell copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"

#include "typedefs.h"
#include "misc.c"
#include "wifi.c"
#include "mqtt.c"
#include "nvs.c"

#include "button.h"
#include "led.h"
#include "esp_rgb_led.h"
#include "i2c_bus.h"
#include "at24cs0x.h"
#include "tpl5010.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define RGB_LED_SET_RED()			esp_rgb_led_set(&rgb_led, 63, 0, 0)
#define RGB_LED_SET_GREEN()		esp_rgb_led_set(&rgb_led, 0, 63, 0)
#define RGB_LED_SET_BLUE()		esp_rgb_led_set(&rgb_led, 0, 0, 63)

/* RTOS task priorities */
#define TASK_RECONNECT_PRIORITY			(tskIDLE_PRIORITY + 1)
#define TASK_OTA_PRIORITY						(tskIDLE_PRIORITY + 2)
#define TASK_WIFI_BUTTON_PRIORITY		(tskIDLE_PRIORITY + 3)
#define TASK_FSM_PRIORITY						(tskIDLE_PRIORITY + 4)
#define TASK_VALVE_BUTTON_PRIORITY	(tskIDLE_PRIORITY + 5)

#define OTA_URL	"https://wit-firmware.s3.amazonaws.com/wit102.bin"

/* Private variables ---------------------------------------------------------*/
/* Tag for debug */
static const char *TAG = "app";

/* Application variables */
static TaskHandle_t reconnect_handle = NULL;
static TaskHandle_t ota_updates_handle = NULL;
static esp_mqtt_client_handle_t mqtt_client;
static bool valve_state = true;

/* MQTT topics */
static char *data_out_topic = NULL;			/* Topic to publish outgoing data */
static char *data_in_topic = NULL;			/* Topic to receive incoming data */
static char *ota_updates_topic = NULL;	/* Topic to receive a notification to perform an OTA update */
static char *online_topic = NULL;				/* Topic to publish the device state */
static char *restore_topic = NULL;			/* Topic to receive a notification to restore the device */
static char *calib_topic = NULL;				/* Topic to receive a notification to perform a calibration process  */

/* Certificates */
extern const uint8_t aws_pem_start[] asm("_binary_aws_pem_start");

/* Device settings */
static char *dev_id = NULL;				/* Device ID */
static char *dev_cert = NULL;			/* Device certificate */
static char *priv_key = NULL;			/* Private key */
static char *topic_parent = NULL;	/* MQTT topic parent */

/* Components instances */
static button_t wifi_button;
static button_t valve_button;
static led_t wifi_led;
static esp_rgb_led_t rgb_led;
static i2c_bus_t i2c_bus;
static at24cs0x_t at24cs02;
static tpl5010_t tpl5010;


/* FSM variables */
fsm_events_e fsm_event;
fsm_states_e fsm_state;
QueueHandle_t fsm_ext_events_queue;

/* Private function prototypes -----------------------------------------------*/
/* Initialization functions */
static esp_err_t valve_init(void);
static void valve_button_cb(void *arg);
static bool valve_set(bool desired_state);

/**/
static esp_err_t fsm_init(void);
static void fsm_task(void * arg);

/* RTOS tasks */
static void wifi_reconnect_task(void * arg);
static void ota_updates_task(void * arg);

/* Event handlers */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data);
static void ip_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data);
static void prov_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data);
static esp_err_t prov_custom_data_handler(uint32_t session_id,
		const uint8_t *inbuf,	ssize_t inlen, uint8_t **outbuf, ssize_t *outlen,
		void *priv_data);
static void mqtt_event_handler(void * arg, esp_event_base_t event_base,
		int32_t event_id, void * event_data);

/* Utils */
static esp_err_t load_device_settings(void);
static void erase_wifi_creds(void *arg);

/* Private user code ---------------------------------------------------------*/
/* main */
void app_main(void) {
	/* Initialize I2C bus */
	ESP_ERROR_CHECK(i2c_bus_init(&i2c_bus,	/* I2C bus instance */
			I2C_NUM_0,													/* I2C port number */
			GPIO_NUM_35, 												/* I2C SDA gpio number */
			GPIO_NUM_36, 												/* I2C SCL gpio number */
			false,       												/* I2C SDA pullup resistor */
			false,       												/* I2C SCL pullup resistor */
			400000));    												/* I2C frequency */

//	/* Initialize AT24CS02 */
	at24cs0x_init(&at24cs02,	/* AT24CS0X instance */
			&i2c_bus,							/* I2C bus instance */
			AT24CS0X_I2C_ADDRESS,	/* I2C device address */
			NULL,									/* I2C custom read function */
			NULL);								/* I2C custom write function */

	/* Initialize TPL5010 */
	ESP_ERROR_CHECK(tpl5010_init(&tpl5010,	/* TPL5010 instance */
			GPIO_NUM_37, 												/* Wake GPIO number */
			GPIO_NUM_38)); 											/* Done GPIO number  */

	/* Initialize buttons */
	ESP_ERROR_CHECK(button_init(&wifi_button, /* Button instance */
			GPIO_NUM_6,														/* Button GPIO number */
			BUTTON_EDGE_FALLING,									/* Button edge interrupt */
			TASK_WIFI_BUTTON_PRIORITY,						/* Button task priority */
			configMINIMAL_STACK_SIZE * 4));				/* Button task stack size */
	button_add_cb(&wifi_button, BUTTON_CLICK_MEDIUM, erase_wifi_creds, NULL);
	button_add_cb(&wifi_button, BUTTON_CLICK_SINGLE, valve_button_cb, NULL);
	button_add_cb(&wifi_button, BUTTON_CLICK_DOUBLE, valve_button_cb, NULL);

//	ESP_ERROR_CHECK(button_init(&valve_button,	/* Button GPIO instance */
//			GPIO_NUM_46,														/* Button GPIO number */
//			BUTTON_EDGE_FALLING,										/* Button edge interrupt */
//			TASK_VALVE_BUTTON_PRIORITY,							/* Button GPIO task priority */
//			configMINIMAL_STACK_SIZE * 4));					/* Button GPIO task stack size */
//	button_add_cb(&valve_button, BUTTON_CLICK_SINGLE, valve_button_cb, NULL);

	/* Initialize rgb_led */
	ESP_ERROR_CHECK(led_init(&wifi_led,	/* LED GPIO instance */
			GPIO_NUM_7));										/* LED GPIO number */

	ESP_ERROR_CHECK(esp_rgb_led_init(&rgb_led,	/* RGB LED GPIO instance */
			GPIO_NUM_45,														/* RGB LED GPIO number */
			1));																		/* RGB LED number */

	/* Initialize NVS */
	ESP_ERROR_CHECK(nvs_init());

	/* Get data from NVS */
	ESP_ERROR_CHECK(load_device_settings());

	/* Initialize Wi-Fi */
	ESP_ERROR_CHECK(wifi_init(wifi_event_handler,
			ip_event_handler,
			prov_event_handler,
			prov_custom_data_handler));

	/* Build MQTT topic names */
	data_out_topic = mqtt_build_topic(topic_parent, "wit102", dev_id, "data_out");
	data_in_topic = mqtt_build_topic(topic_parent, "wit102", dev_id, "data_in");
	ota_updates_topic = mqtt_build_topic(topic_parent, "wit102", dev_id, "ota");
	online_topic = mqtt_build_topic(topic_parent, "wit102", dev_id, "online");
	calib_topic = mqtt_build_topic(topic_parent, "wit102", dev_id, "calib");
	restore_topic = mqtt_build_topic(topic_parent, "wit102", dev_id, "restore");

	/* Initialize valve GPIOs and close it */
	RGB_LED_SET_RED();
	ESP_ERROR_CHECK(valve_init());
	valve_state = valve_set(false);

	/* Initialize FSM */
	ESP_ERROR_CHECK(fsm_init());
}

/* Utils */
static esp_err_t valve_init(void) {
	esp_err_t ret = ESP_OK;

	/* Configure valve GPIOs */
	gpio_config_t gpio_conf = {
			.intr_type = GPIO_INTR_DISABLE,
			.mode = GPIO_MODE_OUTPUT,
			.pin_bit_mask = (1ULL << GPIO_NUM_12) | (1ULL << GPIO_NUM_33),
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.pull_up_en = GPIO_PULLUP_DISABLE
	};
	ret = gpio_config(&gpio_conf);

	if(ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to configure GPIOs");
		return ret;
	}

	/* Set default GPIOs levels */
	ret = gpio_set_level(GPIO_NUM_12, 0);

	if(ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set GPIO level");
		return ret;
	}

	ret = gpio_set_level(GPIO_NUM_33, 0);

	if(ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set GPIO level");
		return ret;
	}

	/* Return ESP_OK */
	return ret;
}

static esp_err_t fsm_init(void) {
	esp_err_t ret = ESP_OK;

	/* Create a queue to receive all external events */
	fsm_ext_events_queue = xQueueCreate(3, sizeof(fsm_events_e));

	if (fsm_ext_events_queue == NULL) {
		ESP_LOGE(TAG, "Error creating queue");
		return ESP_FAIL;
	}

	/* Set the FSM event and state to its default values */
	fsm_event = FSM_INIT_SYSTEM_EVENT;
	fsm_state = FSM_IDLE_STATE;

	/* Create the RTOS task where the FSM will run*/
	if (xTaskCreate(fsm_task,
			"FSM Task",
			configMINIMAL_STACK_SIZE * 6,
			NULL,
			TASK_FSM_PRIORITY,
			NULL) != pdPASS) {
		ESP_LOGE(TAG, "Error creating task");
		return ESP_FAIL;
	}

	return ret;
}

/* RTOS tasks */
static void wifi_reconnect_task(void *arg) {
	uint8_t reconnection_attemps = 0;
	TickType_t last_time_wake = xTaskGetTickCount();

	for (;;) {
		/* Try connecting to Wi-Fi router using stored credentials. If
		 * connection is successful then the task delete itself, in other cases
		 * this function is executed again
		 */
		ESP_LOGI(TAG, "Failed to reconnect. Retrying...");

		esp_wifi_connect();

		if (reconnection_attemps++ > 15) { /* todo: put in macros */
			ESP_LOGE(TAG, "Max number of recconection attemps reached");
			reconnection_attemps = 0;
			reset_device(NULL);
		}

		/* Wait CONFIG_WIFI_RECONNECT_TIME to try to reconnect */
		vTaskDelayUntil(&last_time_wake, pdMS_TO_TICKS(CONFIG_WIFI_RECONNECT_TIME));
	}
}

static void ota_updates_task(void *arg) {
	esp_mqtt_client_stop(mqtt_client);

	if (ota_update(OTA_URL, (char *)aws_pem_start) != ESP_OK) {
		esp_mqtt_client_start(mqtt_client); /* todo: make sure to start mqtt client or restart */
	}
	else {
		reset_device(NULL);
	}

	/* Delete task */
	ota_updates_handle = NULL;
	vTaskDelete(NULL);
}

static void fsm_task(void *arg) {
	fsm_states_e current_state = FSM_IDLE_STATE;
	fsm_states_e next_state = FSM_IDLE_STATE;
	fsm_events_e ext_event;
	BaseType_t queue_status;

	for (;;) {
		switch (current_state) {
			case FSM_IDLE_STATE:
				printf("FSM_IDLE_STATE\r\n");
				/* Wait indefinitely for an external event */
				queue_status = xQueueReceive(fsm_ext_events_queue, &ext_event, portMAX_DELAY);

				/* Set the FSM next state according current external event */
				if (queue_status == pdPASS) {
					if (ext_event == FSM_MQTT_OPEN_EVENT) {
						next_state = FSM_SET_ACTUATORS_STATE;
					}
					else if (ext_event == FSM_MQTT_CLOSE_EVENT) {
						next_state = FSM_SET_ACTUATORS_STATE;
					}
					else if (ext_event == FSM_BUTTON_EVENT) {
						next_state = FSM_SET_ACTUATORS_STATE;
					}
					else if (ext_event == FSM_PENDING_DATA_EVENT) {
						next_state = FSM_SEND_DATA_STATE;
					}
					else {
						next_state = FSM_IDLE_STATE;
					}
				}

				break;

			case FSM_SET_ACTUATORS_STATE:
				printf("FSM_SET_ACTUATORS_STATE\r\n");
				/*  */
				if (ext_event == FSM_MQTT_OPEN_EVENT) {
					valve_state = valve_set(true);
					RGB_LED_SET_GREEN();
					next_state = FSM_SEND_DATA_STATE; /* Or FSM_SEND_DATA_STATE to confirm successful execution */
				}
				else if (ext_event == FSM_MQTT_CLOSE_EVENT) {
					valve_state = valve_set(false);
					RGB_LED_SET_RED();
					next_state = FSM_SEND_DATA_STATE; /* Or FSM_SEND_DATA_STATE to confirm successful execution */
				}
				else if (ext_event == FSM_BUTTON_EVENT) {
					valve_state = valve_set(!valve_state);

					if (valve_state) {
						RGB_LED_SET_GREEN();
					}
					else {
						RGB_LED_SET_RED();
					}

					next_state = FSM_SEND_DATA_STATE;
				}
				else {
					next_state = FSM_IDLE_STATE;
				}

				break;

			case FSM_SEND_DATA_STATE:
				printf("FSM_SEND_DATA_STATE\r\n");

				/* Send to MQTT broker the new valve state */
				esp_mqtt_client_publish(mqtt_client, data_out_topic, valve_state? "1" : "0", 1, 0, 0);

				next_state = FSM_IDLE_STATE;

				break;

			default:
				/* Set the FSM state to its default value */
				next_state = FSM_IDLE_STATE;

				break;
		}

		/* Change the current state only if the next state is different */
		if (current_state != next_state) {
			current_state = next_state;
		}
	}

	/* Wait for 100 ms */
	vTaskDelay(pdMS_TO_TICKS(100));
}

/* Event handlers */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
							   int32_t event_id, void * event_data) {
	switch (event_id) {
		case WIFI_EVENT_STA_START: {
			ESP_LOGI(TAG, "WIFI_EVENT_STA_START");

			break;
		}

		case WIFI_EVENT_STA_CONNECTED: {
			ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED");

			/* Delete task to reconnect to AP */
			if (reconnect_handle != NULL) {
				vTaskDelete(reconnect_handle);
				reconnect_handle = NULL;
			}

			break;
		}

		case WIFI_EVENT_STA_DISCONNECTED: {
			ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");

			/* Set Wi-Fi LED status */
			led_set_fade(&wifi_led, 100, 200);

			/* Create wifi_reconnect_task to reconnect the device to the AP */
			if (reconnect_handle == NULL) {
				if (xTaskCreate(wifi_reconnect_task,
						"Reconnect Wi-Fi Task",
						configMINIMAL_STACK_SIZE * 3,
						NULL,
						TASK_RECONNECT_PRIORITY,
						&reconnect_handle) != pdPASS) {
					ESP_LOGI(TAG, "Error creating task");
					error_handler();
				}
			}

	        break;
		}

		default:
			ESP_LOGI(TAG, "Other Wi-Fi event");
			break;
	}
}

static void ip_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void * event_data) {
	switch (event_id) {
		case IP_EVENT_STA_GOT_IP: {
			ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP");

			/* Set Wi-Fi status LED */
			led_set_continuous(&wifi_led, 100);

			/* Initialize MQTT */
			mqtt_init(&mqtt_client,
					CONFIG_MQTT_BROKER_URL,
					(const char*)aws_pem_start,
					dev_cert,
					priv_key,
					online_topic,
					mqtt_event_handler);

			break;
		}

		default: {
			ESP_LOGI(TAG, "Other IP event");
			break;
		}
	}
}

static void prov_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
	switch (event_id) {
		case WIFI_PROV_START: {
			ESP_LOGI(TAG, "WIFI_PROV_START");

			/* Set Wi-Fi LED status */
			led_set_fade(&wifi_led, 100, 500);

			break;
		}

		case WIFI_PROV_CRED_RECV: {
			ESP_LOGI(TAG, "WIFI_PROV_CRED_RECV");

			wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
			ESP_LOGI(TAG, "Credentials received, SSID: %s & Password: %s",
					(const char *) wifi_sta_cfg->ssid, (const char *) wifi_sta_cfg->password);

			break;
		}

		case WIFI_PROV_CRED_SUCCESS: {
			ESP_LOGI(TAG, "WIFI_PROV_CRED_SUCCESS");
			break;
		}

		case WIFI_PROV_END: {
			ESP_LOGI(TAG, "WIFI_PROV_END");

			/* De-initialize manager once provisioning is finished */
			wifi_prov_mgr_deinit();

			/* Wait for 500 ms and restart the device */
			vTaskDelay(pdMS_TO_TICKS(500));
			esp_restart();

			break;
		}

		case WIFI_PROV_CRED_FAIL: {
			ESP_LOGI(TAG, "WIFI_PROV_CRED_FAIL");

			/* Erase any stored Wi-Fi credentials  */
			ESP_LOGI(TAG, "Erasing Wi-Fi credentials");

			esp_err_t ret;

			nvs_handle_t nvs_handle;
			ret = nvs_open("nvs.net80211", NVS_READWRITE, &nvs_handle);

			if(ret == ESP_OK) {
				nvs_erase_all(nvs_handle);
			}

			/* Close NVS */
			ret = nvs_commit(nvs_handle);
			nvs_close(nvs_handle);

			if(ret == ESP_OK) {
				/* Restart device */
				esp_restart();
			}

			break;
		}

		case WIFI_PROV_DEINIT: {
			ESP_LOGI(TAG, "WIFI_PROV_DEINIT");
			break;
		}
		default: {
			ESP_LOGI(TAG, "Other event");
			break;
		}
	}
}

static void mqtt_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void * event_data) {
	esp_mqtt_event_handle_t event = event_data;

	switch(event_id) {
		case MQTT_EVENT_CONNECTED: {
			ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

			/* Publish to user define alive topic */
			esp_mqtt_client_publish(mqtt_client, online_topic, "1", 1, 0, 0);
			ESP_LOGI(TAG, "Sent publish successful");

			/* Subscribe to user defined valve state and ota notification topics */
			esp_mqtt_client_subscribe(mqtt_client, data_in_topic, 0);
			ESP_LOGI(TAG, "Sent subscribe successful to %s topic", data_in_topic);

			esp_mqtt_client_subscribe(mqtt_client, ota_updates_topic, 0);
			ESP_LOGI(TAG, "Sent subscribe successful to %s topic", ota_updates_topic);

			break;
		}

		case MQTT_EVENT_DISCONNECTED: {
			ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");

			break;
		}

		case MQTT_EVENT_DATA: {
			ESP_LOGI(TAG, "MQTT_EVENT_DATA");

			/*  */
			char string[256];

			/* Copy to buffer and print topic */
			sprintf(string, "%.*s", event->topic_len, event->topic);
			ESP_LOGI(TAG, "topic=%s", string);

			if(!strcmp(string, data_in_topic)) {
				/* Copy to buffer and print data */
				sprintf(string, "%.*s", event->data_len, event->data);
				ESP_LOGI(TAG, "data=%s", string);

				/* Put the correct external event in queue */
				fsm_events_e event;

				if(!strcmp("1", string)) {
					event = FSM_MQTT_OPEN_EVENT;
					printf("FSM_MQTT_OPEN_EVENT\r\n");
					xQueueSend(fsm_ext_events_queue, &event, 0);
				}
				else if(!strcmp("0", string)) {
					event = FSM_MQTT_CLOSE_EVENT;
					printf("FSM_MQTT_CLOSE_EVENT\r\n");
					xQueueSend(fsm_ext_events_queue, &event, 0);
				}
				else {
					ESP_LOGI(TAG, "Invalid state");
				}
			}
			else if(!strcmp(string, ota_updates_topic)) {
				/* Copy to buffer and print data */
				sprintf(string, "%.*s\r\n", event->data_len, event->data);
				ESP_LOGI(TAG, "data=%s", string);

				/* Create task to OTA updates */
				if(ota_updates_handle == NULL) {
					if(xTaskCreate(ota_updates_task,
							"OTA Task",
							configMINIMAL_STACK_SIZE * 6,
							NULL,
							TASK_OTA_PRIORITY,
							&ota_updates_handle) != pdPASS) {
						ESP_LOGI(TAG, "Error creating task");
						error_handler();
					}
				}
			}
			else {
				ESP_LOGE(TAG, "Error in topic name");
			}

			break;
		}

		default:
			ESP_LOGI(TAG, "Other MQTT event");
			break;
	}
}

static esp_err_t prov_custom_data_handler(uint32_t session_id, const uint8_t *inbuf,
	ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data) {
	esp_err_t ret = ESP_OK;

	if (inbuf) {
		ESP_LOGI(TAG, "Received data: %.*s", inlen, (char *)inbuf);

		/* Copy the input buffer */
		topic_parent = realloc(topic_parent, inlen + 1);
		sprintf(topic_parent, "%.*s", inlen, (char *)inbuf);

		/* Save topic parent in NVS  */
		ESP_ERROR_CHECK(nvs_save_string("certs", "topic_parent", topic_parent));
	}

	*outbuf = (uint8_t *)strdup(dev_id);

	if(*outbuf == NULL) {
			ESP_LOGE(TAG, "System out of memory");
			return ESP_ERR_NO_MEM;
	}

	*outlen = strlen(dev_id) + 1; /* +1 for NULL terminating byte */

	/* Return ESP_OK */
	return ret;
}

/* Utils */
static esp_err_t load_device_settings(void) {
	esp_err_t ret = ESP_OK;

	/* Load device certificate */
	ret = nvs_load_string("certs", "dev_cert", &dev_cert);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to load dev_cert from NVS");
		return ret;
	}

	/* Load device ID */
	ret = nvs_load_string("certs", "dev_id", &dev_id);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to load dev_id from NVS");
		return ret;
	}

	/* Load device key */
	ret = nvs_load_string("certs", "priv_key", &priv_key);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to load priv_key from NVS");
		return ret;
	}

	/* Load MQTT parent topic key */
	ret = nvs_load_string("certs", "topic_parent", &topic_parent);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to load topic_parent from NVS");

		/* Assign the topic_parent default value */
		topic_parent = malloc(MQTT_PARENT_TOPIC_MAX_SIZE);

		if (topic_parent == NULL) {
			ESP_LOGE(TAG, "Failed to allocate memory for topic_parent");
			return ESP_ERR_NO_MEM;
		}

		sprintf(topic_parent, "user");

		ret = ESP_OK;
	}

	return ret;
}

static void erase_wifi_creds(void *arg) {
	/* Erase any stored Wi-Fi credential  */
	ESP_LOGI(TAG, "Erasing Wi-Fi credentials...");

	esp_err_t ret;

	nvs_handle_t nvs_handle;
	ret = nvs_open("nvs.net80211", NVS_READWRITE, &nvs_handle);

	if(ret == ESP_OK) {
		nvs_erase_all(nvs_handle);
	}

	/* Close NVS */
	ret = nvs_commit(nvs_handle);
	nvs_close(nvs_handle);

	if(ret == ESP_OK) {
		/* Restart device */
		ESP_LOGI(TAG, "Restarting device...");
		esp_restart();
	}
}

static void valve_button_cb(void *arg) {
	fsm_events_e event = FSM_BUTTON_EVENT;
	printf("FSM_BUTTON_EVENT\r\n");
	xQueueSend(fsm_ext_events_queue, &event, 0);
}

static bool valve_set(bool desired_state) {
	bool state = valve_state;

	if (desired_state != valve_state) {
		/* Set GPIOs state */
		gpio_set_level(GPIO_NUM_12, desired_state);
		gpio_set_level(GPIO_NUM_33, !desired_state);

		/* Wait for a time to open/close the valve */
		vTaskDelay(pdMS_TO_TICKS(CONFIG_VALVE_OUTPUT_TIME));

		/* Disable valve pins */
		gpio_set_level(GPIO_NUM_12, 0);
		gpio_set_level(GPIO_NUM_33, 0);

		state = desired_state;
	}

	return state;
}
/***************************** END OF FILE ************************************/
