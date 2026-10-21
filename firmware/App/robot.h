#ifndef ROBOT_H
#define ROBOT_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

typedef enum{
	STOP                    = 0,
	
	BOOST_AFTER_START       = 10,
	NORMAL_TRACE            = 11,               //11
	LARGE_LEFT_TURN         = 12,            //12
	LARGE_RIGHT_TURN        = 13,           //13
	

	CROSS_LINE_ONE          = 21,             //21
	CROSS_LINE_TWO          = 22,             //22
	TRACE_AFTER_CROSS_LINE  = 23,     //23

	PREPARE_TO_TURN_LEFT    = 31,       //31
	TURN_LEFT_90            = 32,               //32 
	END_TURN_LEFT_90        = 33,           //33
	
	
	PREPARE_TO_TURN_RIGHT   = 41,      //41
	TURN_RIGHT_90           = 42,              //42   
	END_TURN_RIGHT_90       = 43,          //43 


	RIGHT_HALF_LANE_ONE     = 51,          //51
	RIGHT_HALF_LANE_TWO     = 52,          //52
	CHANGE_RIGHT_LANE       = 53,            //53
	STABLIZE_RIGHT	        = 54,				//54
	END_CHANGE_RIGHT_LANE   = 55,        //55
//  AFTER_CHANGE_RIGHT_LANE = 56,

	LEFT_HALF_LANE_ONE      = 61,           //61
	LEFT_HALF_LANE_TWO      = 62,           //62
	CHANGE_LEFT_LANE        = 63,             //63
	STABLIZE_LEFT           = 64,
	END_CHANGE_LEFT_LANE    = 65,         //65
 // AFTER_CHANGE_LEFT_LANE = 66,   
					
	BOOST_AFTER_CHANGE_LANE = 70, 
	BOOST_AFTER_TURN90      = 80,

	CORNER_ADVANCE          = 90,   // 90-degree corner: drive on until the wheels reach it
	CORNER_STOP             = 91,   // short stop at the corner
	CORNER_TURN             = 92,   // turn 90 degrees towards the side that saw the line
	//ACCELLERATION  =  81,
			
}run_case;


typedef struct{
	uint32_t cnt1;
	uint32_t line_lost_since;
	bool line_lost;
	int8_t last_track_angle;
	float dist;
	bool ready,run;
	uint8_t sw_mask;
	uint16_t sensor_value[8];
	uint8_t sensor_mask;
	int8_t servo_calib;
	int8_t robot_speed;
	run_case runcase;
	uint8_t running;
	TIM_HandleTypeDef* robot_tim1;
	TIM_HandleTypeDef* robot_tim2;
	TIM_HandleTypeDef* robot_tim3;
	TIM_HandleTypeDef* robot_tim4;
	DMA_HandleTypeDef* robot_dma;
	ADC_HandleTypeDef* robot_adc;
} Robot; 


void robot_init(Robot *robot, ADC_HandleTypeDef* hadc, DMA_HandleTypeDef* hdma_adc, TIM_HandleTypeDef* htim[]);
void timer_handle(Robot *robot);
void loop(Robot *robot);
void run(Robot *robot);
void handle_and_speed(int8_t angle, int8_t speed);

bool check_crossline(void);
bool check_rightline(void);
bool check_leftline(void);
uint8_t sensor_mask(uint8_t sensor, uint8_t MASK);


///-------
bool d_check_crossline(uint8_t sensor);	
bool d_check_rightline(uint8_t sensor);
bool d_check_leftline(uint8_t sensor);


/// 

uint8_t debounce_check(uint8_t sensor);



#endif
