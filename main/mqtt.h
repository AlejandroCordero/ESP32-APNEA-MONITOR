#ifndef __MQTT_H__
#define __MQTT_H__

#include "esp_err.h"

//*****************************************************************************
//      DEFINICIONES
//*****************************************************************************

#define MQTT_BROKER_URL      "mqtt://192.168.1.69:1883"
#define MQTT_TOPIC_SUBSCRIBE_BASE      CONFIG_EXAMPLE_MQTT_TOPIC_SUBSCRIBE_BASE
#define MQTT_TOPIC_PUBLISH_BASE  CONFIG_EXAMPLE_MQTT_TOPIC_PUBLISH_BASE


//*****************************************************************************
//      PROTOTIPOS DE FUNCIONES
//*****************************************************************************

esp_err_t mqtt_app_start(const char* url);
void mqtt_publish_data(const char *topic, const char *payload);


#endif //  __MQTT_H__
