#include "sensor_debounce.h"
#include "rgb.h"

RGB rgb1;

void debounce_init(DebounceBuffer *buffer){
	uint8_t i = 0;
	for(i = 0; i < DEBOUNCE_COUNT; i++)
	{
		 buffer->sensor_data[i] = 0;
	}
	for (i =0; i < 8; i++){
		buffer -> sensor_array[i] = 0;
	}
	buffer -> index = 0;
}

uint8_t extract_bit(uint8_t data, uint8_t bit_index){
   return (data >> bit_index) & 1;
}

uint8_t combine_bit(uint8_t sensor_array[8]){
	uint8_t combined_value = 0;
	for (int i = 0; i < 8; i++) {
    combined_value |= (sensor_array[i] << i);  // Bitwise OR operation to set bits in combined_value
	}
	return combined_value;
}

int compare(const void *a, const void *b) {
    return (*(int *)a - *(int *)b);
}

uint8_t majority_vote(DebounceBuffer *buffer, int sensor_index) {
    int count = 0;
    for (int i = 0; i < DEBOUNCE_COUNT; i++) {
        count += extract_bit(buffer->sensor_data[i], sensor_index);
    }
    return (count > DEBOUNCE_COUNT / 2) ? 1 : 0;
}

uint8_t median_vote(DebounceBuffer *buffer, int sensor_index) {
    int values[DEBOUNCE_COUNT];
    for (int i = 0; i < DEBOUNCE_COUNT; i++) {
        values[i] = extract_bit(buffer->sensor_data[i], sensor_index);
    }
    qsort(values, DEBOUNCE_COUNT, sizeof(int), compare);
    return values[DEBOUNCE_COUNT / 2];
}

uint8_t mode_vote(DebounceBuffer *buffer, int sensor_index) {
    int counts[2] = {0, 0};
    for (int i = 0; i < DEBOUNCE_COUNT; i++) {
        counts[extract_bit(buffer->sensor_data[i], sensor_index)]++;
    }
    return (counts[1] > counts[0]) ? 1 : 0;
}

// MUC DICH KIEM TRA CHONG NHIEM CUA LINE TAI VI TRI 90 DO VA CHUYEN LAN VOI NHIEU LAN XAC DINH TANG DO CHINH XAC CUA KET QUA
// HAN CHE CHECK SAI 90 DO THANH CHUYEN LAN
uint8_t  check_lines(voting_types type, ADC *adc, DebounceBuffer *buffer){

	switch(type)
	{
		case MAJORITY_VOTE:{ // LAY RA GIA TRI XUAT HIEN NHIEU NHAT
			uint8_t i = 0;
			for(i = 0; i < DEBOUNCE_COUNT; i++)
			{
				 buffer -> sensor_data[i] = get_sensor_mask(adc);
			}
			
			for ( i =0; i < 8; i++){
				 buffer -> sensor_array[i] =  majority_vote(buffer, i);
			}		
		return combine_bit(buffer -> sensor_array);
		}	
		
		case MEDIAN_VOTE:{ // LAY RA GIA TRI TRUNG BINH CUA CAM BIEN
			uint8_t i = 0;
			for(i = 0; i < DEBOUNCE_COUNT; i++)
			{
				 buffer -> sensor_data[i] = get_sensor_mask(adc);
			}
			for (int i =0; i < 8; i++){
				 buffer -> sensor_array[i] =  median_vote(buffer, i);
			}
		return	combine_bit(buffer -> sensor_array);
		}
		
		case MODE_VOTE:{ // LAY RA GIA TRI SAU
			uint8_t i = 0;
			for(i = 0; i < DEBOUNCE_COUNT; i++)
			{
				 buffer -> sensor_data[i] = get_sensor_mask(adc);
			}
			for (int i =0; i < 8; i++){
				 buffer -> sensor_array[i] =  mode_vote(buffer, i);
			}
			return	combine_bit(buffer -> sensor_array);
		}
		
		default:{
			rgb_setcolor(&rgb1, WHITE);
			break;
			//return NORMAL_TRACE; 
		}
	}
	return NORMAL_TRACE;
}

//END
