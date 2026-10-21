#include "check_line.h"

#define					MASK4_4					0xff    /* oooooooo	  11111111									*/
#define         MASK3_3         0xe7    /* oooxxooo   11100111                  */

bool check_crossline(ADC *line_sensor){
	
	bool ret = 0;
	if(
			sensor_mask(get_sensor_mask(line_sensor), MASK3_3) == 0xe7 || //1110 0111
	    sensor_mask(get_sensor_mask(line_sensor), 0xc3)    == 0xc3   //1100 0011
	)
	{
		ret = 1;
	}
	return ret;
}	

bool check_rightline(ADC *line_sensor){
	
	bool ret = 0;
	  if(sensor_mask(get_sensor_mask(line_sensor), MASK4_4) == 0x1f //00011111
	   ||sensor_mask(get_sensor_mask(line_sensor), MASK4_4) == 0x0f //00001111
 	   ||sensor_mask(get_sensor_mask(line_sensor), MASK4_4) == 0x3f //00111111
 	   ||sensor_mask(get_sensor_mask(line_sensor), MASK4_4) == 0x7f //01111111
	)
	{
		ret = 1;
	}
	return ret;
}

bool check_leftline(ADC *line_sensor){
	
	bool ret = 0;
	if(sensor_mask(get_sensor_mask(line_sensor), MASK4_4) == 0xf8		//11111000
	 ||sensor_mask(get_sensor_mask(line_sensor), MASK4_4) == 0xf0		//11110000
 	 ||sensor_mask(get_sensor_mask(line_sensor), MASK4_4) == 0xfc		//11111100
   ||sensor_mask(get_sensor_mask(line_sensor), MASK4_4) == 0xfe   //11111110
	)
	{
		ret = 1;
	}
	return ret;
}

uint8_t sensor_mask(uint8_t sensor, uint8_t MASK){
	return ( sensor & MASK);
}


//END

