//Include standard lib headers
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


//Include FreeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

//Include ESP submodules headers.
#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/gpio.h"

//Include own project  headers
#include "gpio_leds.h"
#include "mqtt.h"

//FROZEN JSON parsing/fotmatting library header
#include "frozen.h"

//****************************************************************************
//      VARIABLES GLOBALES STATIC
//****************************************************************************

static const char *TAG = "MQTT_CLIENT";
static esp_mqtt_client_handle_t client=NULL;
static TaskHandle_t senderTaskHandler=NULL;

//****************************************************************************
// Funciones.
//****************************************************************************

//static void mqtt_sender_task(void *pvParameters);


// callback that will handle MQTT events. Will be called by  the MQTT internal task.
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    switch (event->event_id) {
//        case MQTT_EVENT_CONNECTED:
//            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
//            msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_SUBSCRIBE_BASE, 0);
//            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
//            xTaskCreate(mqtt_sender_task, "mqtt_sender", 4096, NULL, 5, &senderTaskHandler); //Crea la tarea MQTT sennder
//            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            //Deber�amos destruir la tarea que env�a....
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
        {
        	//Para poder imprimir el nombre del topic lo tengo que copiar en una cadena correctamente terminada....
        	char topic_name[event->topic_len+1]; //Esto va a la pila y es potencialmente peligroso si el nombre del topic es grande....
        	strncpy(topic_name,event->topic,event->topic_len);
        	topic_name[event->topic_len]=0; //a�ade caracter de terminacion al final.

        	ESP_LOGI(TAG, "MQTT_EVENT_DATA: Topic %s",topic_name);

        	bool booleano;
        	if(json_scanf(event->data, event->data_len, "{ redLed: %B }", &booleano)==1)
        	{
        		ESP_LOGI(TAG, "redLed: %s", booleano ? "true":"false");

        		gpio_set_level(BLINK_GPIO_1, booleano);
        	}
        }
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        case MQTT_EVENT_BEFORE_CONNECT:
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}


//static void mqtt_sender_task(void *pvParameters)
//{
//	char buffer[100]; //"buffer" para guardar el mensaje. Me debo asegurar que quepa...
//	bool booleano=0;
//
//	while (1)
//	{
//		vTaskDelay(configTICK_RATE_HZ);
//		struct json_out out1 = JSON_OUT_BUF(buffer, sizeof(buffer)); // Inicializa la estructura que gestiona el buffer.
//																	// Hay que hacerlo cada vez para empezar a rellenar desde el principio
//																	// si quiero acumular varios "printf" en la misma cadena, no reinicio out1....
//		json_printf(&out1," { button: %B }",booleano);
//		booleano=!booleano;
//
//		int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC_PUBLISH_BASE, buffer, 0, 0, 0); //al utilizar la biblioteca Frozen, buffer es una cadena correctamente terminada con el caracter 0.
//																								//as� que puedo no indicar la longitud (cuarto par�metro vale 0).
//																								// Si fuese un puntero a datos binarios arbitr�rios, tendr�a que indicar la longitud de los datos en el cuarto par�metro de la funci�n.
//		ESP_LOGI(TAG, "sent successful, msg_id=%d: %s", msg_id, buffer);
//	}
//}

esp_err_t mqtt_app_start(const char* url)
{
	esp_err_t error;

	if (client==NULL){

		esp_mqtt_client_config_t mqtt_cfg = {
				.broker.address.uri = MQTT_BROKER_URL,
		};
		if(url[0] != '\0'){
			mqtt_cfg.broker.address.uri= url;
		}


		client = esp_mqtt_client_init(&mqtt_cfg);
		esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
		error=esp_mqtt_client_start(client);
		return error;
	}
	else {

		ESP_LOGE(TAG,"MQTT client already running");
		return ESP_FAIL;
	}
}


void mqtt_publish_data(const char *topic, const char *payload)
{
    if (client) {
        int msg_id = esp_mqtt_client_publish(client, topic, payload, 0, 0, 0);
        ESP_LOGI(TAG, "Publicado mensaje ID=%d: %s", msg_id, payload);
    } else {
        ESP_LOGW(TAG, "Cliente MQTT no conectado aún");
    }
}

