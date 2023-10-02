/**
  ******************************************************************************
  * @file           : mqtt.c
  * @author         : Mauricio Barroso Benavides
  * @date           : Sep 2, 2023
  * @brief          : todo: write brief
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2023 Mauricio Barroso Benavides
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
#include "esp_err.h"
#include "esp_log.h"
#include "mqtt_client.h"

/* Private macros ------------------------------------------------------------*/
#define MQTT_PARENT_TOPIC_MAX_SIZE	(128)
#define MQTT_CHILD_TOPIC_MAX_SIZE		(64)
#define MQTT_DEV_ID_MAX_SIZE				(64)

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Exported functions definitions --------------------------------------------*/
esp_err_t mqtt_init(esp_mqtt_client_handle_t *mqtt_client,
		const char *broker_uri, const char *broker_cert,
		const char *dev_cert, const char *dev_key, const char *lwt_topic,
		esp_event_handler_t mqtt_event_handler) {
	esp_err_t ret = ESP_OK;

	ESP_LOGI("mqtt", "Initializing MQTT client...");

	/* Fill MQTT client configuration and initialize */
	const esp_mqtt_client_config_t mqtt_config = {
			.broker = {
					.address.uri = broker_uri,
					.verification.certificate = broker_cert
			},
			.credentials = {
					.authentication = {
						.certificate = dev_cert,
						.key = dev_key
					},
			},
			.session = {
					.last_will = {
							.topic = lwt_topic,
							.msg = "0",
							.msg_len = 1,
							.qos = 0
					},
			},
	};

	*mqtt_client = esp_mqtt_client_init(&mqtt_config);

	/* Register MQTT event handler */
	if (mqtt_client != NULL) {
		ret = esp_mqtt_client_register_event(*mqtt_client,
				MQTT_EVENT_ANY,
				mqtt_event_handler,
				NULL);

		if (ret != ESP_OK) {
			ESP_LOGE("mqtt", "Failed to register event handler");
			return ret;
		}
	}
	else {
		ESP_LOGE("mqtt", "Failed to initialize MQTT client");
		return ESP_FAIL;
	}

	/* Start MQTT client */
	ret = esp_mqtt_client_start(*mqtt_client);

	/* Return ESP_OK */
	return ret;
}

char *mqtt_build_topic(char *parent, char *dev, char *id, char *child) {
	char *topic = NULL;

	/* Allocate memory for the full topic name */
	topic = malloc(MQTT_PARENT_TOPIC_MAX_SIZE + MQTT_DEV_ID_MAX_SIZE + MQTT_CHILD_TOPIC_MAX_SIZE);

	if (topic == NULL) {
		ESP_LOGE("mqtt", "Failed to allocate memory");
		return NULL;
	}

	/* Build de the full topic name */
	if (sprintf(topic, "%s/%s/%s/%s", parent, dev, id, child) < 0) {
		ESP_LOGE("mqtt", "Failed build the topic name");
		return NULL;
	}

	return topic;
}

/* Private function definitions ----------------------------------------------*/

/***************************** END OF FILE ************************************/
