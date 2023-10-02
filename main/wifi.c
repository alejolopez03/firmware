/**
  ******************************************************************************
  * @file           : wifi.c
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
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_softap.h"

/* Private macros ------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static char *get_device_service_name(const char *ssid_prefix);

/* Exported functions definitions --------------------------------------------*/
esp_err_t wifi_init(esp_event_handler_t wifi_event_handler,
		esp_event_handler_t ip_event_handler,
		esp_event_handler_t prov_event_handler,
		protocomm_req_handler_t prov_custom_data_event_handler) {
	esp_err_t ret = ESP_OK;

	ESP_LOGI("wifi", "Initializing Wi-Fi...");

	/* Initialize stack TCP/IP */
	ret = esp_netif_init();

	if (ret != ESP_OK) {
		return ret;
	}

	/* Create event loop */
	ret = esp_event_loop_create_default();

	if (ret != ESP_OK) {
		return ret;
	}

	/* Create netif instances */
	esp_netif_create_default_wifi_sta();
	esp_netif_create_default_wifi_ap();

	/* Initialize Wi-Fi driver*/
	wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&wifi_config);

	/* Declare event handler instances for Wi-Fi and IP */
	esp_event_handler_instance_t instance_any_wifi;
	esp_event_handler_instance_t instance_got_ip;
	esp_event_handler_instance_t instance_any_prov;

	/* Register Wi-Fi, IP and SmartConfig event handlers */
	ret = esp_event_handler_instance_register(WIFI_EVENT,
			ESP_EVENT_ANY_ID,
			wifi_event_handler,
			NULL,
			&instance_any_wifi);

	if (ret != ESP_OK) {
		ESP_LOGE("wifi", "Failed to regitster Wi-Fi event handler");
		return ret;
	}

	ret = esp_event_handler_instance_register(IP_EVENT,
			IP_EVENT_STA_GOT_IP,
			ip_event_handler,
			NULL,
			&instance_got_ip);

	if (ret != ESP_OK) {
		ESP_LOGE("wifi", "Failed to regitster IP event handler");
		return ret;
	}

	ret = esp_event_handler_instance_register(WIFI_PROV_EVENT,
			ESP_EVENT_ANY_ID,
			prov_event_handler,
			NULL,
			&instance_any_prov);

	if (ret != ESP_OK) {
		ESP_LOGE("wifi", "Failed to regitster provisioining event handler");
		return ret;
	}

	/* Set Wi-Fi mode and config */
	ret = esp_wifi_set_mode(WIFI_MODE_APSTA);

	if (ret != ESP_OK) {
		return ret;
	}

	/* Start Wi-Fi */
	ret = esp_wifi_start();

	if (ret != ESP_OK) {
		return ret;
	}

	/**/
	ret = esp_wifi_set_ps(WIFI_PS_NONE);

	if (ret != ESP_OK) {
		ESP_LOGE("wifi", "Failed to setting power save type ");
		return ret;
	}

	/* Check if are Wi-Fi credentials provisioned */
  wifi_prov_mgr_config_t prov_config = {
  		.scheme = wifi_prov_scheme_softap,
			.scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
			.app_event_handler = WIFI_PROV_EVENT_HANDLER_NONE
  };

  ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_config));

  /* Check if are Wi-Fi credentials provisioned */
  bool provisioned = false;
	ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

	if (provisioned) {
		ESP_LOGI("wifi", "Already provisioned. Connecting to AP...");

		/* We don't need the manager as device is already provisioned,
		* so let's release it's resources */
		wifi_prov_mgr_deinit();

		/* Try connecting to Wi-Fi router using stored credentials */
		esp_wifi_set_mode(WIFI_MODE_STA);
		esp_wifi_connect();
	}
	else {
		ESP_LOGI("wifi", "Starting provisioning...");

		/* Create endpoint */
		wifi_prov_mgr_endpoint_create("custom-data");

		/* Get SoftAP SSID name */
		char *ap_name_prov = get_device_service_name("PROV_");
		ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_1,
				NULL,
				ap_name_prov,
				NULL));

		free(ap_name_prov);

		/* Register previous created endpoint */
		wifi_prov_mgr_endpoint_register("custom-data", prov_custom_data_event_handler, NULL);
	}

	/* Return ESP_OK */
	return ret;
}

/* Private function definitions ----------------------------------------------*/
static char *get_device_service_name(const char *ssid_prefix) {
	char *name = NULL;

	name = (char *)malloc((strlen(ssid_prefix) + 6 + 1) * sizeof(*name));

	uint8_t eth_mac[6];
	esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
	sprintf(name, "%s%02X%02X%02X", ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);

	return name;
}

/***************************** END OF FILE ************************************/
