/*
 * tcp_server.c
 *
 *  Created on: 5 dic. 2020
 *      Author: jcgar
 */

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "driver/adc.h"
#include "gpio_leds.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#include "protocol.h"

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "TCPSERVER";



//Lee EXACTAMENDE size datos... para poder implementar el protocolo
static int16_t ReadFrame(int SockID, void *ptr,int16_t size)
{
	int16_t estado,i;

	i=size;
	while(i>0)
	{
		estado = recv(SockID, ptr,i, 0); //Esta funcion finaliza en cuanto llegan datos. Los tengo que ir acumulando hasta recibir la trama
		if (estado>0)
		{
			ptr+=estado;
			i-=estado;
		}
		else
		{
			return estado;
		}
	}
	return size;
}

//Bucle que procesa el paquete
static int tcp_server_process_data(const int sock)
{
	int len;
	TipoPaquete header;

	do {
		len = ReadFrame(sock,(void*)&header,sizeof(header));	//Se queda experando hasta recibir el tamaño exacto
		if (len < 0) {
			ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
		} else if (len == 0) {
			ESP_LOGW(TAG, "Connection closed");
		} else {
			switch (header.comando)
			{
				case COMANDO_PING:
					//paquete ping, responde con el mismo comando
					ESP_LOGI(TAG, "llega comando de PING");
					len=send(sock,&header,sizeof(header),0);
					if (len<=0) return len;
					break;
				case COMANDO_LED:
					gpio_set_level(BLINK_GPIO_1, header.parametro.bits.bit0);
					break;

				default:
				{
					//Tipo de trama no implementada, responde con comando rechazado
					header.parametro.value=header.comando;
					header.comando=COMANDO_RECHAZADO;
					len=send(sock,&header,sizeof(header),0);
					if (len<=0) return len;
				}
			}
		}
	} while (len > 0);
	return len;
}


static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    //inicialización del socket. API similar a la de LINUX y otros SO.
#ifdef CONFIG_EXAMPLE_IPV4
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
    struct sockaddr_in6 dest_addr;
    bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(PORT);
    addr_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
    inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");
        //Pasamos a esperar conexiones
        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Convert ip address to string
        if (source_addr.sin6_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        } else if (source_addr.sin6_family == PF_INET6) {
            inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        //Si llegamos aquí es que la conexion se ha creado. Pasamos a recibir
        tcp_server_process_data(sock);	//Bucle de recepcion

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void initTCPServer (void)
{
	xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL); //Crea la tarea TCP Server-
}

