/*
 * sensors.h
 *
 *  Created on: 9 Jun 2025
 *      Author: aleja
 */

#ifndef MAIN_SENSORS_H_
#define MAIN_SENSORS_H_

#define BUFFER_SIZE 128

#include "max30102_api.h"

void sensors_init(void);
void sensor_data_processor(void *pvParameters);
void sensor_data_reader(void *pvParameters);
void fill_buffers_data();


#endif /* MAIN_SENSORS_H_ */
