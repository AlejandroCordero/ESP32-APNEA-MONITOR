/*
 * sensors.c
 *
 * Refactor: 12 Jun 2025
 * - Manejo de stack overflow ajustado
 * - Integradas tareas de sensores: TSL2561, SCD4x, monitor digital y MAX30102
 * - Eliminado monitor ADC analógico, no necesario para MAX30102
 * - Añadido cálculo básico de BPM y SpO2 en MAX30102 con validaciones
 */

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <limits.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sensors.h"
#include <float.h>  // Para FLT_MAX y FLT_MIN
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "sensors.h"
#include "max30102_api.h"
#include "algorithm.h"
#include "i2c_api.h"
//#include "mqtt_wifi.h"
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#include "mqtt.h"


/* ---------- Configuración I2C ---------- */
#define I2C_PORT            I2C_NUM_0
#define I2C_SCL_PIN         22
#define I2C_SDA_PIN         21
#define I2C_CLK_HZ          100000

/* ---------- Configuración TSL2561 ---------- */
#define TSL2561_ADDR        0x39
#define CMD                 0x80
#define REG_CONTROL         0x00
#define REG_TIMING          0x01
#define REG_DATA_0_LOW      0x0C
#define REG_DATA_1_LOW      0x0E
#define POWER_ON            0x03
#define POWER_OFF           0x00
#define INTEGRATE_402MS     0x02

/* ---------- Configuración SCD4x ---------- */
#define SCD4X_ADDR          0x62

/* ---------- Configuración GPIO para monitor digital ---------- */
#define DIGITAL_GPIO            GPIO_NUM_4

/* ---------- Configuración MAX30102 ---------- */
#define MAX30102_ADDR           0x57
#define REG_INTR_ENABLE_1       0x02
#define REG_INTR_ENABLE_2       0x03
#define REG_FIFO_WR_PTR         0x04
#define REG_FIFO_OVF_COUNTER    0x05
#define REG_FIFO_RD_PTR         0x06
#define REG_FIFO_DATA           0x07
#define REG_FIFO_CONFIG         0x08
#define REG_MODE_CONFIG         0x09
#define REG_SPO2_CONFIG         0x0A
#define REG_LED1_PA             0x0C
#define REG_LED2_PA             0x0D
#define MAX30102_MODE_SPO2      0x03

/* Parámetros para muestreo y cálculo de BPM/SpO2 */
#define MAX30102_SAMPLE_COUNT 250  // 10s de ventana a 40ms
#define MAX30102_SAMPLE_PERIOD_MS 40  // 25 Hz de muestreo
#define BPM_FILTER_SIZE 5

#define DELAY_AMOSTRAGEM 40

int32_t red_data = 0;
int32_t ir_data = 0;
int32_t red_data_buffer[BUFFER_SIZE];
int32_t ir_data_buffer[BUFFER_SIZE];
double auto_correlationated_data[BUFFER_SIZE];

char *data = NULL;

TaskHandle_t processor_handle = NULL;
TaskHandle_t sensor_reader_handle = NULL;

static const char *TAG = "SENSORS";
static bool i2c_initialized = false;
static SemaphoreHandle_t i2c_mutex = NULL;

/* ---------- Inicialización I2C ---------- */
static esp_err_t i2c_master_init(void)
{
    if (!i2c_mutex) {
        i2c_mutex = xSemaphoreCreateMutex();
        if (!i2c_mutex) {
            ESP_LOGE(TAG, "No se pudo crear el mutex de I2C");
            return ESP_FAIL;
        }
    }
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Timeout al tomar mutex de I2C");
        return ESP_FAIL;
    }
    if (i2c_initialized) {
        xSemaphoreGive(i2c_mutex);
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Inicializando I2C en SDA=%d, SCL=%d...", I2C_SDA_PIN, I2C_SCL_PIN);
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN, // 21
        .scl_io_num = I2C_SCL_PIN, // 22
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLK_HZ,
        .clk_flags = 0,
    };
    esp_err_t err = i2c_param_config(I2C_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config falló: %s", esp_err_to_name(err));
        xSemaphoreGive(i2c_mutex);
        return err;
    }
    err = i2c_driver_install(I2C_PORT, cfg.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install falló: %s", esp_err_to_name(err));
        xSemaphoreGive(i2c_mutex);
        return err;
    }
    i2c_initialized = true;
    ESP_LOGI(TAG, "I2C inicializado");
    xSemaphoreGive(i2c_mutex);
    return ESP_OK;
}

/* ---------- TSL2561 ---------- */
static esp_err_t tsl2561_write(uint8_t reg, uint8_t val)
{
    uint8_t cmd = CMD | reg;
    return i2c_master_write_to_device(I2C_PORT, TSL2561_ADDR, (uint8_t[]){cmd, val}, 2, pdMS_TO_TICKS(100));
}
static esp_err_t tsl2561_read(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t cmd = CMD | reg;
    return i2c_master_write_read_device(I2C_PORT, TSL2561_ADDR, &cmd, 1, data, len, pdMS_TO_TICKS(100));
}
esp_err_t tsl2561_read_lux(uint32_t *lux)
{
    if (!lux) return ESP_ERR_INVALID_ARG;
    if (!i2c_initialized) return ESP_ERR_INVALID_STATE;
    if (tsl2561_write(REG_CONTROL, POWER_ON) != ESP_OK) return ESP_FAIL;
    if (tsl2561_write(REG_TIMING, INTEGRATE_402MS) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(450));
    uint8_t buf[4];
    esp_err_t err = tsl2561_read(REG_DATA_0_LOW, buf, sizeof(buf));
    if (err != ESP_OK) return err;
    uint16_t ch0 = buf[1] << 8 | buf[0];
    uint16_t ch1 = buf[3] << 8 | buf[2];
    tsl2561_write(REG_CONTROL, POWER_OFF);
    if (ch0 == 0) return ESP_ERR_INVALID_STATE;
    float ratio = (float)ch1 / ch0;
    float lux_f;
    if (ratio <= 0.50f)       lux_f = 0.0304f * ch0 - 0.062f  * ch0 * powf(ratio, 1.4f);
    else if (ratio <= 0.61f)  lux_f = 0.0224f * ch0 - 0.031f  * ch1;
    else if (ratio <= 0.80f)  lux_f = 0.0128f * ch0 - 0.0153f * ch1;
    else if (ratio <= 1.30f)  lux_f = 0.00146f* ch0 - 0.00112f* ch1;
    else                      lux_f = 0.0f;
    *lux = (uint32_t)lux_f;
    return ESP_OK;
}
void tsl2561_task(void *arg)
{
    ESP_LOGI(TAG, "tsl2561_task iniciada");
    if (i2c_master_init() != ESP_OK) { ESP_LOGE(TAG, "I2C falla en tsl2561"); vTaskDelete(NULL); }
    while (1) {
        uint32_t lux;
        if (tsl2561_read_lux(&lux) == ESP_OK) {
			ESP_LOGI(TAG, "Lux: %u lx", (unsigned int)lux);
			
			char payload[128];
			snprintf(payload, sizeof(payload), "{\"Lux: \":%u}",  (unsigned int)lux);
            mqtt_publish_data("sensors/esp32", payload);
			
		}
			
        else ESP_LOGW(TAG, "Error al leer TSL2561");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* ---------- Monitor GPIO digital ---------- */
void digital_monitor_task(void *arg)
{
    ESP_LOGI(TAG, "digital_monitor_task iniciada");
    gpio_set_direction(DIGITAL_GPIO, GPIO_MODE_INPUT);
    bool state_prev = gpio_get_level(DIGITAL_GPIO);
    while (1) {
        bool state = gpio_get_level(DIGITAL_GPIO);
        if (state && !state_prev) {
			ESP_LOGI(TAG, "DIGITAL ON");
			
			char payload[128];
            snprintf(payload, sizeof(payload), "{\"Noise: \":1}");
            mqtt_publish_data("sensors/esp32", payload);
		} 
        else if (!state && state_prev) {
			char payload[128];
			snprintf(payload, sizeof(payload), "{\"Noise: \":0}");
            mqtt_publish_data("sensors/esp32", payload);
            
			ESP_LOGI(TAG, "DIGITAL OFF");
		} 
        state_prev = state;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---------- SCD4x ---------- */
static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}
static esp_err_t scd4x_cmd(uint16_t cmd)
{
    uint8_t buf[2] = { cmd >> 8, cmd & 0xFF };
    return i2c_master_write_to_device(I2C_PORT, SCD4X_ADDR, buf, 2, pdMS_TO_TICKS(100));
}
static esp_err_t scd4x_read_measurement(uint16_t *co2, float *temp_c, float *rh)
{
    uint8_t buf[9];
    esp_err_t ret = i2c_master_read_from_device(I2C_PORT, SCD4X_ADDR, buf, 9, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;
    for (int i = 0; i < 3; ++i) {
        if (crc8(&buf[i*3], 2) != buf[i*3+2]) { ESP_LOGW(TAG, "CRC fallo en campo %d", i); return ESP_ERR_INVALID_RESPONSE; }
    }
    *co2 = (buf[0]<<8)|buf[1];
    uint16_t raw_temp = (buf[3]<<8)|buf[4];
    uint16_t raw_rh   = (buf[6]<<8)|buf[7];
    *temp_c = -45 + 175*((float)raw_temp/65535.0f);
    *rh     = 100*((float)raw_rh/65535.0f);
    return ESP_OK;
}
void scd4x_task(void *arg)
{
    ESP_LOGI(TAG, "scd4x_task iniciada");

    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "Fallo en inicializacion I2C");
        vTaskDelete(NULL);
    }

    // Escaneo I2C para detectar dispositivos
    ESP_LOGI(TAG, "Escaneando bus I2C...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Dispositivo I2C detectado en 0x%02X", addr);
        }
    }

    ESP_LOGI(TAG, "Inicializando sensor SCD4x...");

    // Enviar STOP_MEASUREMENT
    ESP_LOGI(TAG, "Enviando comando STOP_MEASUREMENT (0x3F86)");
    if (scd4x_cmd(0x3F86) != ESP_OK) {
        ESP_LOGW(TAG, "Comando STOP_MEASUREMENT fallo");
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // Enviar START_PERIODIC_MEASUREMENT
    ESP_LOGI(TAG, "Enviando comando START_PERIODIC_MEASUREMENT (0x21B1)");
    if (scd4x_cmd(0x21B1) != ESP_OK) {
        ESP_LOGE(TAG, "Comando START_PERIODIC_MEASUREMENT fallo. Verificar conexion al sensor");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Esperando 5 segundos para obtener primera medicion valida...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        uint16_t co2;
        float temp, rh;

        esp_err_t ret = scd4x_read_measurement(&co2, &temp, &rh);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "CO2: %u ppm, Temp: %.2f C, RH: %.2f%%", co2, temp, rh);

            char payload[128];
            snprintf(payload, sizeof(payload), "{\"co2\":%u,\"temp\":%.1f,\"humidity\":%.1f}", co2, temp, rh);
            mqtt_publish_data("sensors/esp32", payload);
        } else {
            ESP_LOGW(TAG, "Error al leer datos del sensor (ret=0x%x)", ret);
        }
    }
}


/* ---------- MAX30102 ---------- */

max_config max30102_configuration = {

		.INT_EN_1.A_FULL_EN         = 1,
		.INT_EN_1.PPG_RDY_EN        = 1,
		.INT_EN_1.ALC_OVF_EN        = 0,
		.INT_EN_1.PROX_INT_EN       = 0,

		.INT_EN_2.DIE_TEMP_RDY_EN   = 0,

		.FIFO_WRITE_PTR.FIFO_WR_PTR = 0,

		.OVEF_COUNTER.OVF_COUNTER   = 0,

		.FIFO_READ_PTR.FIFO_RD_PTR  = 0,

		.FIFO_CONF.SMP_AVE          = 0b010,  //média de 4 valores
		.FIFO_CONF.FIFO_ROLLOVER_EN = 1,      //fifo rollover enable
		.FIFO_CONF.FIFO_A_FULL      = 0,      //0

		.MODE_CONF.SHDN             = 0,
		.MODE_CONF.RESET            = 0,
		.MODE_CONF.MODE             = 0b011,  //SPO2 mode

		.SPO2_CONF.SPO2_ADC_RGE     = 0b01,   //16384 nA(Escala do DAC)
		.SPO2_CONF.SPO2_SR          = 0b001,  //200 samples per second
		.SPO2_CONF.LED_PW           = 0b10,   //pulso de 215 uS do led.

		.LED1_PULSE_AMP.LED1_PA     = 0x24,   //CORRENTE DO LED1 25.4mA
		.LED2_PULSE_AMP.LED2_PA     = 0x24,   //CORRENTE DO LED2 25.4mA

		.PROX_LED_PULS_AMP.PILOT_PA = 0X7F,

		.MULTI_LED_CONTROL1.SLOT2   = 0,      //Desabilitado
		.MULTI_LED_CONTROL1.SLOT1   = 0,      //Desabilitado

		.MULTI_LED_CONTROL2.SLOT4   = 0,      //Desabilitado
		.MULTI_LED_CONTROL2.SLOT3   = 0,      //Desabilitado
};

void fill_buffers_data()
{
	for(int i = 0; i < BUFFER_SIZE; i++){
		read_max30102_fifo(&red_data, &ir_data);
		ir_data_buffer[i] = ir_data;
		red_data_buffer[i] = red_data;
		//printf("%d\n", red_data);
		ir_data = 0;
		red_data = 0;
		vTaskDelay(pdMS_TO_TICKS(DELAY_AMOSTRAGEM));
	}
}


void max30102_task(void *arg)
{	
	max30102_init(&max30102_configuration);
	init_time_array();
	uint64_t ir_mean;
	uint64_t red_mean;
	float temperature;
	double r0_autocorrelation;
	size_t size;

	for(;;){
		//vTaskDelay(pdMS_TO_TICKS(100));
		fill_buffers_data();
		temperature = get_max30102_temp();
		remove_dc_part(ir_data_buffer, red_data_buffer, &ir_mean, &red_mean);
		remove_trend_line(ir_data_buffer);
		remove_trend_line(red_data_buffer);
		double pearson_correlation = correlation_datay_datax(red_data_buffer, ir_data_buffer);
		int heart_rate = calculate_heart_rate(ir_data_buffer, &r0_autocorrelation, auto_correlationated_data);

		printf("\n");
		printf("HEART_RATE %d\n", heart_rate);
		printf("correlation %f\n", pearson_correlation);
		printf("Temperature %f\n", temperature);

		if(pearson_correlation >= 0.7){
			double spo2 = spo2_measurement(ir_data_buffer, red_data_buffer, ir_mean, red_mean);
			printf("SPO2 %f\n", spo2);

	        //size = asprintf(&data, "{\"mac\": \"%02x%02f%02x%0f%02x%02x\", \"spo2\":%f, \"heart_rate\":%d}",MAC2STR(sta_mac), spo2, heart_rate);
			//mqtt_publish(data, size);
		}
		printf("\n");  
	}
}


//void sensor_data_reader(void *pvParameters)
//{
//    uint8_t sta_mac[6] = {0};
//    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
//	i2c_init();
//	vTaskDelay(pdMS_TO_TICKS(100));	
//
//#if DEBUG
////		for(int i = 0; i < BUFFER_SIZE; i++){
////			printf("%d", (int)ir_data_buffer[i]);
////			printf(" ");
////			printf("%d", (int)red_data_buffer[i]);
////			printf(" ");
////			printf("%f\n", auto_correlationated_data[i]);
////		}
//#endif
//}






/* ---------- sensors_init ---------- */
void sensors_init(void)
{
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "Fallo inicializando I2C en sensors_init");
        return;
    }
    BaseType_t res;
    //res = xTaskCreate(tsl2561_task, "tsl2561_task", 4096, NULL, 5, NULL);
    //if (res != pdPASS) ESP_LOGE(TAG, "No se pudo crear tsl2561_task");
    // SCD40 roto, esperando otro
    //res = xTaskCreate(scd4x_task, "scd4x_task", 4096, NULL, 5, NULL);
    //if (res != pdPASS) ESP_LOGE(TAG, "No se pudo crear scd4x_task");
    //res = xTaskCreate(digital_monitor_task, "digital_monitor", 4096, NULL, 5, NULL);
    //if (res != pdPASS) ESP_LOGE(TAG, "No se pudo crear digital_monitor_task");
    res = xTaskCreate(max30102_task, "max30102_task", 8192, NULL, 5, NULL);
    if (res != pdPASS) ESP_LOGE(TAG, "No se pudo crear max30102_task");
}
