//*****************************************************************************

#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

//*****************************************************************************
//      DEFINICIONES
//*****************************************************************************

//#define WIFI_MODO_STATION           0
//#define WIFI_MODO_SOFTAP            1


//*****************************************************************************
//      PROTOTIPOS DE FUNCIONES
//*****************************************************************************

esp_err_t bluetooth_start_scan(void);

esp_err_t bluetooth_stop_scan(void);

//esp_err_t bluetooth_start_adv(const char* nombre);
//esp_err_t bluetooth_stop_adv(void);

esp_err_t bluetooth_init(void);

#endif //  __BLUETOOTH_H__
