#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
//#include "comandos.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_blufi_api.h"

#include "bluetooth.h"

#define BT_BD_ADDR_STR         "%02x:%02x:%02x:%02x:%02x:%02x"
#define BT_BD_ADDR_HEX(addr)   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]

//****************************************************************************
//      VARIABLES LOCALES
//****************************************************************************

static const char *TAG = "_BLUETOOTH";

static bool scan_ongoing = false;
static bool adv_ongoing = false;

static esp_ble_scan_params_t ble_scan_params = 
{	
	.scan_type              = BLE_SCAN_TYPE_ACTIVE,
	.own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
	.scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
	.scan_interval          = 0x500,
	.scan_window            = 0x30		
};

static esp_ble_adv_data_t adv_data = {
  .include_name = true,
};

//static esp_ble_adv_params_t ble_adv_params =
//{
//	.adv_int_min = 0x20,
//	.adv_int_max = 0x40,
//	.adv_type = ADV_TYPE_NONCONN_IND,
//	.own_addr_type  = BLE_ADDR_TYPE_PUBLIC,
//	.channel_map = ADV_CHNL_ALL,
//	.adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
//};

//****************************************************************************
//      DEFINICIÃ“N DE FUNCIONES
//****************************************************************************

//Manejador del evento del protocolo GAP bluetooth
static void gap_callback_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	ESP_LOGI(TAG, "Received a GAP event: %d", event);
	esp_err_t status = 0;	

	switch (event) 
	{
		case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
		{
			// This procedure keep the device scanning the peer device which advertising on the air.
			//the unit of the duration is second
			uint32_t duration = 30;
			status = esp_ble_gap_start_scanning(duration); 
			if (status != ESP_OK) 
			{
				ESP_LOGE(TAG, "esp_ble_gap_start_scanning: rc=%d", status);
			}
		}
		break;

		case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
		{
			//scan start complete event to indicate scan start successfully or failed
			if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
			{
          	  ESP_LOGE(TAG, "Scan start failed");
			}
			else
			{
				ESP_LOGW(TAG, "---------------- Scan start");
			}
		}
		break;
		
		case ESP_GAP_BLE_SCAN_RESULT_EVT:
		{	
			switch (param->scan_rst.search_evt)
			{
				case ESP_GAP_SEARCH_INQ_RES_EVT:
				{
					char address[30];
					snprintf(address,30, BT_BD_ADDR_STR, BT_BD_ADDR_HEX(param->scan_rst.bda));
					 ESP_LOGW(TAG, "Device found (bda): %s", address);
					 ESP_LOGW(TAG, "RSSI                : %d", param->scan_rst.rssi);
					 ESP_LOGW(TAG, "SCAN LEN: %d RSP LEN. %d",param->scan_rst.adv_data_len,param->scan_rst.scan_rsp_len);
					 ESP_LOG_BUFFER_HEXDUMP(TAG,param->scan_rst.ble_adv,param->scan_rst.adv_data_len,ESP_LOG_WARN);
					 ESP_LOG_BUFFER_HEXDUMP(TAG,param->scan_rst.ble_adv+param->scan_rst.adv_data_len,param->scan_rst.scan_rsp_len,ESP_LOG_WARN);



//					// Scans for the "Complete name" (looking inside the payload buffer)
					uint8_t len;
					if (param->scan_rst.scan_rsp_len)
					{
						uint8_t *data = esp_ble_resolve_adv_data(param->scan_rst.ble_adv+param->scan_rst.adv_data_len, ESP_BLE_AD_TYPE_NAME_CMPL, &len);
						ESP_LOGW(TAG, "len: %d, %.*s", len, len, data);
					}

					break;
				}
				case ESP_GAP_SEARCH_INQ_CMPL_EVT:
				{
					// Scan is done.
					 ESP_LOGW(TAG, "Scan finished");
					 scan_ongoing=false;

//					if(scan_active)
//					{
//						// The next 5 codelines automatically restarts the scan.
//						uint32_t duration = 5;
//						status = esp_ble_gap_start_scanning(duration);
//						if (status != ESP_OK)
//						{
//							ESP_LOGE(TAG, "esp_ble_gap_start_scanning: rc=%d", status);
//						}
//					}
					break;
				}
				default:
					break;
			}
		}
		break;

		case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
		{
			if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
			{
				ESP_LOGE(TAG, "Scan stop failed");
			}
			else 
			{
				ESP_LOGI(TAG, "Stop scan successfully");
			}
		}

		case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		{
			if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
			{
				ESP_LOGE(TAG, "Adv stop failed");
			}
			else 
			{
				ESP_LOGI(TAG, "Stop adv successfully");
			}
		}
		break;
				
		default:
        break;
	}
} // gap_callback_handler

esp_err_t bluetooth_start_scan(void)
{
	esp_err_t status;	
	
	scan_ongoing = true;

	// This function is called to set scan parameters.	
	status = esp_ble_gap_set_scan_params(&ble_scan_params);
	if (status != ESP_OK) 
	{
		ESP_LOGE(TAG, "esp_ble_gap_set_scan_params: rc=%d", status);
		return ESP_FAIL;
	}

	return ESP_OK ;
}

esp_err_t bluetooth_stop_scan(void)
{
	esp_err_t status;
	
	scan_ongoing = false;
	
	status = esp_ble_gap_stop_scanning();
	if (status != ESP_OK) 
	{
		ESP_LOGE(TAG, "esp_ble_gap_stop_scanning: rc=%d", status);
		return ESP_FAIL;
	}

	return ESP_OK ;
}

//esp_err_t bluetooth_start_adv(const char* nombre)
//{
//	esp_err_t status;
//
//	adv_active = true;
//
//	if(nombre[0] != '\0')
//		adv_data.include_name = true;
//	else
//		adv_data.include_name = false;
//
//	ESP_ERROR_CHECK(esp_ble_gap_set_device_name(nombre));
//	ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
//
//	// This function is called to set scan parameters.
//	status = esp_ble_gap_start_advertising(&ble_adv_params);
//	if (status != ESP_OK)
//	{
//		ESP_LOGE(TAG, "esp_ble_gap_start_advertising: rc=%d", status);
//		return ESP_FAIL;
//	}
//
//	return ESP_OK ;
//}
//esp_err_t bluetooth_stop_adv(void)
//{
//	esp_err_t status;
//
//	adv_active = false;
//
//	status = esp_ble_gap_stop_advertising();
//	if (status != ESP_OK)
//	{
//		ESP_LOGE(TAG, "esp_ble_gap_stop_advertising: rc=%d", status);
//		return ESP_FAIL;
//	}
//
//	return ESP_OK ;
//}

esp_err_t bluetooth_init(void)
{
    esp_err_t status;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT)); //Vamos a utilizar solo el modo BLE, así que liberamos memoria del modo clásico

	// Initialize BT controller to allocate task and other resource. 
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	if (esp_bt_controller_init(&bt_cfg) != ESP_OK) 
	{
		ESP_LOGE(TAG, "Bluetooth controller initialize failed");
		return ESP_FAIL; 
	}

	// Enable BT controller (BLE mode only)
	if (esp_bt_controller_enable(ESP_BT_MODE_BLE) != ESP_OK)
	{
		ESP_LOGE(TAG, "Bluetooth controller enable failed");
		return ESP_FAIL; 
	}

	ESP_LOGI(TAG, "Bluetooth Controller Enabled");

	// Init and alloc the resource for bluetooth, must be prior to every bluetooth stuff
	status = esp_bluedroid_init(); 
	if (status != ESP_OK)
	{ 
		ESP_LOGE(TAG, "%s init bluetooth failed\n", __func__); 
		return ESP_FAIL; 
	} 
	
	// Enable bluetooth, must after esp_bluedroid_init()
	status = esp_bluedroid_enable(); 
	if (status != ESP_OK) 
	{ 
		ESP_LOGE(TAG, "%s enable bluetooth failed\n", __func__); 
		return ESP_FAIL; 
	} 

	ESP_LOGI(TAG, "Bluetooth stack initialized");

	// This function is called to occur gap event, such as scan result.
	//register the scan callback function to the gap module
	status = esp_ble_gap_register_callback(gap_callback_handler);
	if (status != ESP_OK) 
	{
		ESP_LOGE(TAG, "esp_ble_gap_register_callback: rc=%d", status);
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "GAP callback registered");

	return ESP_OK;
}
