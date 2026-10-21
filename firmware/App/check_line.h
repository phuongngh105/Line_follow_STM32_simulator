#ifndef CHECK_LINE_H
#define CHECK_LINE_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include "adc.h"

bool check_crossline(ADC *line_sensor);
bool check_rightline(ADC *line_sensor);
bool check_leftline(ADC *line_sensor);
uint8_t sensor_mask(uint8_t sensor, uint8_t MASK);

#endif
