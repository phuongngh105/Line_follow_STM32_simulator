#ifndef SENSOR_DEBOUNCE_H
#define SENSOR_DEBOUNCE_H
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include "adc.h"
#include "robot.h" 
#include <stdio.h>
#include <stdlib.h>


#define         MASK3_3         0xe7    /* oooxxooo   11100111                  */
#define					MASK4_4					0xff    /* oooooooo	  11111111									*/
#define         DEBOUNCE_COUNT  15  // Normally, tunning the number from  KHAI BAO GIA TRI DAU VAO SO LAN KIEM TRA <15


typedef struct{
	 uint8_t sensor_data[DEBOUNCE_COUNT];
	 uint8_t sensor_array[8];
   int index;
} DebounceBuffer;


typedef enum{
	MAJORITY_VOTE = 0,
	MEDIAN_VOTE   = 1,
	MODE_VOTE     = 2,
	
} voting_types;

void debounce_init(DebounceBuffer *buffer);
uint8_t extract_bit(uint8_t byte, uint8_t bit_index);
uint8_t combine_bit(uint8_t sensor_array[8]);
uint8_t check_lines(voting_types type, ADC *adc, DebounceBuffer *buffer);

uint8_t majority_vote(DebounceBuffer *buffer, int sensor_index);
uint8_t median_vote(DebounceBuffer *buffer, int sensor_index);
uint8_t mode_vote(DebounceBuffer *buffer, int sensor_index);
int compare(const void *a, const void *b);
#endif
