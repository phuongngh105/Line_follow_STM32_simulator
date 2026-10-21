#include "robot.h" 
#include "buzzer.h"
#include "rgb.h"
#include "servo.h"
#include "motor.h"
#include "sonic.h"
#include "button.h"
#include "dip_switch.h" 
#include "adc.h"
#include "flash.h"
#include "leds.h"
#include "params.h"
#include "calib.h" 
#include "sensor_debounce.h"


Buzzer     buzzer;
RGB        rgb;
Servo      servo;
Motor      motor;
Sonic      sonic;
dip_switch dip1, dip2, dip3;
Button     button1, button2, button3;
ADC        line_sensor;
LEDS       led;
//new
DebounceBuffer d_buffer;


GPIO_TypeDef   *rgb_ports[] = {GPIOB,GPIOA,GPIOA};
uint16_t         rgb_pins[] = {GPIO_PIN_15,GPIO_PIN_8,GPIO_PIN_9};
GPIO_TypeDef *motor_ports[] = {GPIOA,GPIOB};
uint16_t       motor_pins[] = {GPIO_PIN_10,GPIO_PIN_11,GPIO_PIN_1,GPIO_PIN_4,TIM_CHANNEL_3,TIM_CHANNEL_4};
dip_switch           *dip[] = {&dip1,&dip2,&dip3};
Button            *button[] = {&button1,&button2,&button3};
uint16_t         led_pins[] = {GPIO_PIN_12,GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_8, GPIO_PIN_9};//latch,data,clock,oe,sclr

#define SW1 get_button(&button1)
#define SW2 get_button(&button2)
#define SW3 get_button(&button3)
#define DIP get_sw_mask(dip,3)

////// Function to handle Sonic sensor
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){	
	sonic_callback(&sonic);	
}

void robot_init(Robot *robot, ADC_HandleTypeDef* hadc, DMA_HandleTypeDef* hdma_adc, TIM_HandleTypeDef* htim[]){ // ham khoi tao
	
		robot -> robot_tim1  =	htim[0];
	  robot -> robot_tim1 -> Instance -> PSC = 6; // ~ 0.1us
		robot -> robot_tim2  =	htim[1];
		robot -> robot_tim3  =	htim[2];
		robot -> robot_tim4  =	htim[3];
		robot -> robot_dma   =  hdma_adc;
		robot -> robot_adc   =  hadc;
		
		buzzer_init(&buzzer,GPIOB, GPIO_PIN_3);
		rgb_init(&rgb, rgb_ports, rgb_pins);
		servo_init(&servo ,robot -> robot_tim3, TIM_CHANNEL_3);
		motors_init(&motor, robot -> robot_tim2, motor_ports, motor_pins);
	
		sonic_init(&sonic, GPIOA, robot -> robot_tim4, GPIO_PIN_15, GPIO_PIN_12);	
		sw_init(&dip1,GPIOB,GPIO_PIN_5);
		sw_init(&dip2,GPIOB,GPIO_PIN_6);
		sw_init(&dip3,GPIOB,GPIO_PIN_7);
		button_init(&button1,GPIOC,GPIO_PIN_15);
		button_init(&button2,GPIOC,GPIO_PIN_14);
		button_init(&button3,GPIOC,GPIO_PIN_13);
		adc_init(&line_sensor, robot -> robot_adc ,robot -> robot_dma, calib_weight);
		leds_init(&led, GPIOB, led_pins);
		
		//new 
		debounce_init(&d_buffer);
		
		robot -> cnt1 = 0;
		robot -> line_lost_since = 0;
		robot -> line_lost = false;
		robot -> last_track_angle = 0;
		robot -> ready = false;
		robot -> run  = false;
		robot -> dist = 0;
		robot -> sw_mask = 0;
		robot -> sensor_mask = get_sensor_mask(&line_sensor);
		robot -> servo_calib = 0;
		robot -> robot_speed = ROBOT_SPEED;
		robot -> runcase = STOP;	
		robot -> running = 0 ;
		read_servo_center(&servo);
		handle(&servo,0);
}

void loop(Robot *robot){	// vong lap
	
	sw_handle(dip, 3);
	button_multi_handle(button, 3);
	switch( DIP)
	{
		case 0:{ // Calibrate sensor value | 000 | Cam bien hoc mau den va trang
			calib_sensor(&line_sensor,button,&rgb,&buzzer);
			break;
		}
		
		case 1:{ // Save and Load sensor value | 100 | Luu lai gia tri cam bien
			write_read_sensor_value(&line_sensor,button,&rgb,&buzzer);
			break;	
		}
			 
		case 2:{ // Calibrate servo center | 010 | Dieu chinh servo
				calib_servo(&servo,button,&rgb,&buzzer);
			break;
		}
		
		case 3:{ // Check motors | 110 | Kiem tra dong co
			motor_check(&motor,button,&rgb);
			break;
		}

		case 4:{ // Run without Gate bar | 001 | Chay khi an nut
			run(robot);		
			break;
		}		
		case 5:{ // Check Sonic sensor| 101 | kiem tra sonic
			check_sonic(&sonic, &rgb,&buzzer);
			break;
		}
		case 6:{ // Run with gate bar  | 011 | Chay voi thanh chan co ultrasonic
			if (robot -> running == 0){
			RunState state = run_with_gate_bar(&sonic,&rgb, SW1, robot -> ready, robot -> run);
			robot->ready = state.ready;
			robot->run = state.run;}
			run(robot);	
			break;
		}
		case 7:{ // Preserved
			
			
			
		  break;
		} 
		
		default:{
			rgb_setcolor(&rgb, OFF);
			break;
		}	
	}
	
}


void timer_handle(Robot *robot){ // timer
		robot -> cnt1++;
	  robot -> sensor_mask = get_sensor_mask(&line_sensor);
		//robot -> sensor_mask =  check_lines(MEDIAN_VOTE, &line_sensor, &d_buffer);
		led_out(&led, robot -> sensor_mask);
}

/* ---------------------------------------------------------------------------
 * Line-centre tracking: instead of fixed (angle, speed) pairs per sensor mask, use the analog
 * sensor values to compute where the line is (-112 .. +112 = outer sensor, 16 per half-pitch,
 * negative = line on the LEFT) and steer with P + D so the line stays on the middle sensors.
 * Speed drops smoothly as the line moves away from the centre.
 * Tune with the defines below (units: handle() steps, ~1 degree each).
 * ------------------------------------------------------------------------- */
#ifndef TRACK_KP_X10
#define TRACK_KP_X10     110  /* handle steps per half-pitch of error, x10 */
#endif
#ifndef TRACK_KD_X10
#define TRACK_KD_X10     100  /* handle steps per (half-pitch / 16 ms), x10 */
#endif
#ifndef TRACK_SLOW_PCT
#define TRACK_SLOW_PCT   45   /* % of speed removed when the line is under an outer sensor */
#endif
#ifndef TRACK_MIN_SPEED_PCT
#define TRACK_MIN_SPEED_PCT 40
#endif

static int16_t track_position16(uint8_t *seen)
{
	int32_t sum = 0, wsum = 0;
	for (int i = 0; i < 8; i++)
	{
		int32_t black = line_sensor.maxOfMinSensorValue[i];
		int32_t white = line_sensor.minOfMaxSensorValue[i];
		int32_t v     = line_sensor.sensor_value[i];
		int32_t a;
		if (black > white + 20) { a = (v - white) * 256 / (black - white); }
		else                    { a = (v > line_sensor.v_compare[i]) ? 256 : 0; }
		if (a < 0) a = 0;
		if (a > 256) a = 256;
		if (a < 40) a = 0;                       /* ignore floor noise */
		sum  += a;
		wsum += a * (2 * i - 7);
	}
	*seen = (sum > 0);
	return sum > 0 ? (int16_t)(wsum * 16 / sum) : 0;
}

void track_center(Robot *robot, int8_t *angle, int8_t *speed)
{
	static int16_t last_pos = 0, d16 = 0;
	static uint32_t last_ms = 0;
	uint8_t seen;
	int16_t pos = track_position16(&seen);
	uint32_t now = HAL_GetTick();
	if (now - last_ms >= 4)                       /* derivative over >= 4 ms */
	{
		d16 = (int16_t)(((int32_t)(pos - last_pos) * 16) / (int32_t)(now - last_ms));
		last_pos = pos;
		last_ms = now;
	}
	int32_t u = ((int32_t)pos * TRACK_KP_X10 + (int32_t)d16 * TRACK_KD_X10) / 160;
	if (u > 25) u = 25;
	if (u < -25) u = -25;
	*angle = (int8_t)(-u);                         /* line on the right -> steer right (negative) */
	int32_t ap = pos < 0 ? -pos : pos;
	int32_t pct = 100 - (TRACK_SLOW_PCT * ap) / 112;
	if (pct < TRACK_MIN_SPEED_PCT) pct = TRACK_MIN_SPEED_PCT;
	*speed = (int8_t)(robot -> robot_speed * pct / 100);
}


/* ---------------------------------------------------------------------------
 * 90-degree corner: when one whole half of the sensor bar sees the line (left 4 or right 4),
 * keep driving for CORNER_ADVANCE_MS (= distance wheels -> sensor bar / speed), stop briefly,
 * steer fully towards the side that saw the line until the middle sensors find it, then
 * go back to normal tracking.
 * ------------------------------------------------------------------------- */
#ifndef CORNER_90_ENABLE
#define CORNER_90_ENABLE 1
#endif
#ifndef CORNER_ADVANCE_MS
#define CORNER_ADVANCE_MS 330   /* ms at robot_speed to cover the wheel -> sensor-bar distance: TUNE ON THE CAR */
#endif
#ifndef CORNER_STOP_MS
#define CORNER_STOP_MS    120
#endif
#ifndef CORNER_CONFIRM_MS
#define CORNER_CONFIRM_MS 8     /* the half-bar pattern must last this long */
#endif
#ifndef CORNER_TURN_SPEED_PCT
#define CORNER_TURN_SPEED_PCT 45
#endif
#ifndef CORNER_TURN_MIN_MS
#define CORNER_TURN_MIN_MS 150
#endif
#ifndef CORNER_TURN_TIMEOUT_MS
#define CORNER_TURN_TIMEOUT_MS 2500
#endif

static int8_t  corner_dir;       /* +1 = left, -1 = right */
static uint32_t corner_t0;

/* returns +-1 (left/right half, confirmed after CORNER_CONFIRM_MS), +-2 (seen, waiting) or 0 */
static int8_t corner_detect(uint8_t m)
{
	static int8_t seen_dir = 0;
	static uint32_t seen_since = 0;
	int8_t d = 0;
	if      ((m & 0xF0) == 0xF0 && (m & 0x0F) != 0x0F) d = 1;
	else if ((m & 0x0F) == 0x0F && (m & 0xF0) != 0xF0) d = -1;
	if (d == 0) { seen_dir = 0; return 0; }
	if (d != seen_dir) { seen_dir = d; seen_since = HAL_GetTick(); }
	return (HAL_GetTick() - seen_since >= CORNER_CONFIRM_MS) ? d : 2 * d;   /* +-2 = pattern seen, not confirmed yet */
}


void run(Robot *robot){// cac truong hop cua xe
	//rgb_setcolor(&rgb, BLUE);
	if (SW1){ 
	  robot ->  cnt1 = 0;
	  robot ->	runcase = STOP;
		
	}
	
	switch(robot -> runcase){
		case STOP:{
			robot -> line_lost = false;
			robot -> last_track_angle = 0;
			handle_and_speed(0,0);
			if(SW3 || robot -> run)
			{
				beep_long(&buzzer, 100);
				robot -> runcase = NORMAL_TRACE;
				robot -> running = 1;
			}
			
			break;
		}
		
		case NORMAL_TRACE:{//case 11: CHAY BINH THUONG	
				if(get_sensor_mask(&line_sensor) == 0){
					if(!robot -> line_lost){
						robot -> line_lost = true;
						robot -> line_lost_since = HAL_GetTick();
					}

					if(HAL_GetTick() - robot -> line_lost_since < LINE_GAP_BRIDGE_MS){
						handle_and_speed(robot -> last_track_angle, robot -> robot_speed);
					}else{
						handle_and_speed(0, robot -> robot_speed);
					}
					break;
				}
				robot -> line_lost = false;
#if CORNER_90_ENABLE
				{
					int8_t cd = corner_detect(get_sensor_mask(&line_sensor));
					if (cd == 2 || cd == -2) { handle_and_speed(robot -> last_track_angle, robot -> robot_speed); break; }
					if (cd != 0)
					{
						corner_dir = cd;
						corner_t0 = HAL_GetTick();
						robot -> runcase = CORNER_ADVANCE;
						break;
					}
				}
#endif
				if(check_crossline()){robot -> runcase = CROSS_LINE_ONE;break;}
  			if(check_leftline()){robot -> runcase = LEFT_HALF_LANE_ONE;break;}
				if(check_rightline()){robot -> runcase = RIGHT_HALF_LANE_ONE;break;}		
						
			{
				int8_t angle, speed;
				track_center(robot, &angle, &speed);   // PD on the line position, keeps the line at the middle sensors
				robot -> last_track_angle = angle;
				handle_and_speed(angle, speed);
			}
			break;
		}
		
		case CORNER_ADVANCE:{   // drive straight until the wheels are at the corner
			handle_and_speed(0, robot -> robot_speed);
			if (HAL_GetTick() - corner_t0 >= CORNER_ADVANCE_MS)
			{
				set_brake(&motor, 1, 1);
				handle_and_speed(0, 0);
				corner_t0 = HAL_GetTick();
				robot -> runcase = CORNER_STOP;
			}
			break;
		}
		
		case CORNER_STOP:{      // short stop at the corner
			if (HAL_GetTick() - corner_t0 >= CORNER_STOP_MS)
			{
				set_brake(&motor, 0, 0);
				corner_t0 = HAL_GetTick();
				robot -> runcase = CORNER_TURN;
			}
			break;
		}
		
		case CORNER_TURN:{      // spin on the spot towards the line until the middle sensors see it
			uint32_t el = HAL_GetTick() - corner_t0;
			handle(&servo, 0);
			{ int8_t sp = robot -> robot_speed * CORNER_TURN_SPEED_PCT / 100;   /* spin on the spot: wheels in opposite directions */
			  set_speed(&motor, -corner_dir * sp, corner_dir * sp); }
			if ((el >= CORNER_TURN_MIN_MS && (get_sensor_mask(&line_sensor) & 0x18)) || el >= CORNER_TURN_TIMEOUT_MS)
			{
				robot -> last_track_angle = corner_dir * 10;
				robot -> runcase = NORMAL_TRACE;
			}
			break;
		}
		
		case LARGE_LEFT_TURN:{//case 12: RE SANG PHAI
				if(check_crossline()){robot -> runcase = CROSS_LINE_ONE;break;}
  			if(check_leftline()){robot -> runcase = LEFT_HALF_LANE_ONE;break;}
				if(check_rightline()){robot -> runcase = RIGHT_HALF_LANE_ONE;break;}		
				if(robot -> cnt1 > 5)
				{
					if(sensor_mask(robot -> sensor_mask, MASK3_3) == 0x60){
						robot -> runcase = NORMAL_TRACE;
						robot -> cnt1  = 0;
						}
				}	
			
			break;
		}
		
		case LARGE_RIGHT_TURN:{//case 13: RE SANG TRAI
				if(check_crossline()){robot -> runcase = CROSS_LINE_ONE;break;}
  			if(check_leftline()){robot -> runcase = LEFT_HALF_LANE_ONE;break;}
				if(check_rightline()){robot -> runcase = RIGHT_HALF_LANE_ONE;break;}	
				
			if(robot -> cnt1 > 5)
						{
							if(sensor_mask(robot -> sensor_mask, MASK3_3) ==  0x06)
							{
								robot -> runcase = NORMAL_TRACE;
								robot -> cnt1 = 0;
							}
						}
			break;
		}
		
		
		case CROSS_LINE_ONE:{// case 21: DUONG CHEO
			//rgb_setcolor(&rgb, BLUE);
			set_brake(&motor, 1, 1);
			handle_and_speed(0,0);
			robot -> runcase = CROSS_LINE_TWO;
			robot -> cnt1  = 0;
			break;
		}
		
		case CROSS_LINE_TWO:{//case 22: doi 5ms
			if (robot -> cnt1 > 5)
			{
				
				//set_brake(&motor, 1, 1);
				//handle_and_speed(0, robot -> robot_speed*0);
				robot -> runcase = TRACE_AFTER_CROSS_LINE;
			  robot -> cnt1  = 0;
			}
			
			break;
		}	
		
		case TRACE_AFTER_CROSS_LINE:{//case 23: kiem tra 90 trai hay phai
			if(  sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0x80  //10000000
			   || sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0xf0  //11110000
			  || sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0xfc  //11111100
			  || sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0xfe  //11111110
				|| sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0xc0  //11000000
			  ){
				
				// left turn determined -> to the left turn clear processing
				handle_and_speed(0, robot -> robot_speed*0.5);

					robot -> runcase = PREPARE_TO_TURN_LEFT;
				break;
			}
			
			if(sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0x1     //00000001
				 || sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0x3     //00000011
				 || sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0xf     //00001111
				 || sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0x7f     //01111111
			  )
			{
			  //left turn determined -> to the left turn clear processing			
				  robot -> runcase = PREPARE_TO_TURN_RIGHT;
				break;
			}
			//robot_motors.setBrake(0,0);
					switch(sensor_mask(get_sensor_mask(&line_sensor), MASK3_3)) 
			{
				case 0x00: //000xx000
				{
					/* Center -> straight */
					handle_and_speed(0, robot -> robot_speed*0.5);
					break;
				}
				//deviate to the Left//
				case 0x04: //0000 0100
				{
					/* Slight amount left of center ->light turn to right */
					handle_and_speed(-2, robot -> robot_speed*0.5);
					break;
				}
				
				case 0x06: //000 0110
				{
					/* Small amount left of center -> small turn to right */
					handle_and_speed(-5, robot -> robot_speed*0.5);					
					break;
				}					
			
				case 0x02: //000 0010
				{
					/* Medium amount left of center -> medium turn to right */
//					handle(&servo,-6);
//					set_speed(&motor,robot -> robot_speed*1.1,  robot -> robot_speed);
					handle_and_speed(-6,robot -> robot_speed*0.5);
					break;
				}								
				//deviate to the Right//
				case 0x20: //0010 0000
				{
					//rgb_setcolor(&rgb, BLUE);
					handle_and_speed(2, robot -> robot_speed*0.5);
					break;
				}
				case 0x60://0110 0000
				{
					/* Slight amount right of center -> slight turn to left */
					//rgb_setcolor(&rgb, GREEN);
					handle_and_speed(5	, robot -> robot_speed*0.5);
					break;
				}
					
				case 0x40: //1110 0000
				{
					/* Small amount right of center -> small turn to left */
//					handle(&servo,6);
//					set_speed(&motor,robot -> robot_speed,  robot -> robot_speed*1.1);
					handle_and_speed(6,robot -> robot_speed*0.5);
					break;
				}
				default:
				break;
			}

			break;
		}
		
		case PREPARE_TO_TURN_LEFT:{ // CHUAN BI RE TRAI
			if (sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0x00)  // condition to turn left
				{
					//rgb_setcolor(&rgb, GREEN);
					handle(&servo, 25);
					set_speed(&motor, 0,robot -> robot_speed*0.7);
					robot -> cnt1 =0;	
					robot -> runcase  = TURN_LEFT_90;
					
				}			
				break;
		}
		
		case TURN_LEFT_90:{//case 31: doi 80ms
			if (robot -> cnt1 > 50)
			{
				robot -> runcase = END_TURN_LEFT_90;
				robot -> cnt1 = 0;
				
			}
			break;
		}
		
		case END_TURN_LEFT_90:{//case 32: ket thuc xoay trai
			if(sensor_mask(get_sensor_mask(&line_sensor), MASK3_3) == 0x06) //0000 0110
			{				
				//rgb_setcolor(&rgb, GREEN);
				handle(&servo, -2);
				robot -> runcase  = NORMAL_TRACE;
			}
			break;
		}
		case PREPARE_TO_TURN_RIGHT:{ // CHUAN BI RE PHAI
			if (sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0x00)  // condition to turn right
			{
			 // Max angle for turning right ==> Imidiately turn cause => cannot reach 0x60
				handle(&servo, -25);
				set_speed(&motor, robot -> robot_speed*0.7, 0);
				robot -> cnt1  = 0;
				robot -> runcase  = TURN_RIGHT_90;	
			}
			break;
		}
		
		case TURN_RIGHT_90:{//case 41: doi 80ms
			if (robot -> cnt1  > 10)
			{
				robot -> runcase = END_TURN_RIGHT_90;
				robot -> cnt1 = 0;
			}
			break;
		}
		
		case END_TURN_RIGHT_90:{//case 42: ket thuc xoay phai
			if(sensor_mask(get_sensor_mask(&line_sensor), MASK3_3)== 0x60) //01100000
			{
				handle(&servo, 2);
				robot -> runcase = NORMAL_TRACE;
			}
			break;
		}
		
		case RIGHT_HALF_LANE_ONE:{// case 51: cfhuyen lan phai
			//if(check_crossline()){robot -> runcase = CROSS_LINE_ONE;break;}
			robot -> runcase  = CHANGE_RIGHT_LANE;
			break;
		}
		
		case RIGHT_HALF_LANE_TWO:{// case 52: doi 50ms
			if (robot -> cnt1   > 50)
			{
				set_brake(&motor,0,0);
				robot -> runcase  = CHANGE_RIGHT_LANE;		
				
			}
			
			break;
		}
		
		case CHANGE_RIGHT_LANE:{//case 53: chuyen lan phai
		
			if(sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0x00)
			{
				handle_and_speed(-11, 0.8* robot -> robot_speed);
				robot -> runcase  = STABLIZE_RIGHT;
				break;
			}
			switch(sensor_mask(get_sensor_mask(&line_sensor), MASK3_3))
			{
				//Car in the center of the line//
				case 0x00:
				case 0x3c:
				{
					// 001xx100 ->> This case hardly happens
					/* Center -> straight */
					handle_and_speed(0, robot -> robot_speed);
					break;
				}
							

				//deviate to the Left//
				case 0x04:
				{
					//0100 ~ x100
					/* Slight amount left of center -> slight turn to right */
					handle_and_speed(-2, robot -> robot_speed*0.8);
					break;
				}
							
				case 0x06:
				{
					//0110
					/* Small amount left of center -> small turn to right */
					handle_and_speed(-5, robot -> robot_speed*0.6);   //0.8 * speed_run
					break;
				}
							
							
				case 0x02: //0010
				case 0x07://0111
				//case 0x1e:
				{
					/* Medium amount left of center -> medium turn to right */
					handle_and_speed(-7, robot -> robot_speed*0.6); //0.6 * speed_run
					break;
				}
							
				//deviate to the Right//
				case 0x20://0010 0000
				{
					/* Slight amount right of center -> slight turn to left */
					handle_and_speed(2, robot -> robot_speed *0.8);
					break;
				}
							
				case 0x60: //0110 0000
				{
					/* Small amount right of center -> small turn to left */
					handle_and_speed(5, robot -> robot_speed*0.6);//convert_speed(0.8 * speed_run)
					break;
				}
							
				case 0xe0://1110 0000
				case 0x40: //0100 0000
				{
					/* Medium amount right of center -> medium turn to left */
					handle_and_speed(7, robot -> robot_speed * 0.6); //convert_speed( 0.6 * speed_run)
					break;
				}
				case 0xe7:
				{
					if(check_crossline()){robot -> runcase = CROSS_LINE_ONE;break;}
				}
				
				default:
				break;
			}
			break;
		}
		
		case STABLIZE_RIGHT:{ //ON DINH VAO LANE PHAI
			if(sensor_mask(get_sensor_mask(&line_sensor), 0x01) != 0)
			{
				handle_and_speed(0, 0.8*robot -> robot_speed);
				robot -> runcase  = END_CHANGE_RIGHT_LANE;
			}
			break;
		}
		
		case END_CHANGE_RIGHT_LANE:{// case 54: end chuyen lan phai
			if (sensor_mask(get_sensor_mask(&line_sensor), MASK4_0) != 0)
			{
				//robot_rgb.setColor(RGB::OFF);
				robot -> runcase  = NORMAL_TRACE;	
			}
			break;
		}
		
		case LEFT_HALF_LANE_ONE:{//case 61 : chuyen lan trai
			//if(check_crossline()){robot -> runcase = CROSS_LINE_ONE;break;}
			robot -> runcase  = CHANGE_LEFT_LANE;
			break;
		}
		
		case LEFT_HALF_LANE_TWO:{// case 62: doi 50ms
		
			if (robot -> cnt1   > 50)
			{
//				robot_motors.setBrake(0,0);
//				robot_motors.setSpeed(convert_speed(0.4*speed_run),convert_speed(0.4*speed_run));
				robot -> runcase  = CHANGE_LEFT_LANE;
			}
			break;
		}
		
		case CHANGE_LEFT_LANE:{//case 63: chuyen lan trai
		
				// Trace after right half lane
				if(sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0x00)
				{
					handle_and_speed(11, 0.8*robot -> robot_speed);
					robot -> runcase  = STABLIZE_LEFT;
					break;
				}

				switch(sensor_mask(get_sensor_mask(&line_sensor), MASK3_3)){
					//Car in the center of the line//
				case 0x00:
				case 0x3c:
				{
					// 001xx100 ->> This case hardly happens
					/* Center -> straight */
					handle_and_speed(0, robot -> robot_speed);
					break;
				}
							

				//deviate to the Left//
				case 0x04:
				{
					//0100 ~ x100
					/* Slight amount left of center -> slight turn to right */
					handle_and_speed(-2, robot -> robot_speed*0.8);
					break;
				}
							
				case 0x06:
				{
					//0110
					/* Small amount left of center -> small turn to right */
					handle_and_speed(-5, robot -> robot_speed*0.6);   //0.8 * speed_run
					break;
				}
							
							
				case 0x02: //0010
				case 0x07://0111
				//case 0x1e:
				{
					/* Medium amount left of center -> medium turn to right */
					handle_and_speed(-7, robot -> robot_speed*0.6); //0.6 * speed_run
					break;
				}
							
				//deviate to the Right//
				case 0x20://0010 0000
				{
					/* Slight amount right of center -> slight turn to left */
					handle_and_speed(2, robot -> robot_speed *0.8);
					break;
				}
							
				case 0x60: //0110 0000
				{
					/* Small amount right of center -> small turn to left */
					handle_and_speed(5, robot -> robot_speed*0.6);//convert_speed(0.8 * speed_run)
					break;
				}
							
				case 0xe0://1110 0000
				case 0x40: //0100 0000
				{
					/* Medium amount right of center -> medium turn to left */
					handle_and_speed(7, robot -> robot_speed * 0.6); //convert_speed( 0.6 * speed_run)
					break;
				}
				case 0xe7:
					if(check_crossline()){robot -> runcase = CROSS_LINE_ONE;break;}
					
					default:
					break;
				}
				break;
		}
		
		case STABLIZE_LEFT:{ //ON DINH VAO LANE TRAI
			if(sensor_mask(get_sensor_mask(&line_sensor), 0x80) != 0)
			{
				handle_and_speed(0, 0.8 * robot-> robot_speed);
				robot -> runcase  = END_CHANGE_LEFT_LANE;
			}
			break;
		}
		
		case END_CHANGE_LEFT_LANE:{//case 64: end chuyen lan phai
		
					
			if (sensor_mask(get_sensor_mask(&line_sensor), MASK4_0) != 0)
			{
				//robot_rgb.setColor(RGB::OFF);
				robot -> runcase  = NORMAL_TRACE;	
			}
			break;
		}
		


		default:{
		
			rgb_setcolor(&rgb, OFF);
			set_brake(&motor,1,1);
			set_speed(&motor,0,0);
			break;
		}
		
	}
}
void handle_and_speed(int8_t angle, int8_t speed){// HAM HIEU CHINH TOC DO BANH XE VA GOC QUAY SERVO
	int8_t speed1;
	int8_t speed2;
	// Need to calibrate in real case
	if (angle >  25) angle =  25;
	if (angle < -25) angle = -25;
	handle(&servo, angle);
	
	if(angle < 0) // turn right
	{
		angle = -angle;
		speed1 = speed;
		speed2 = speed*coeffR1[angle]/100;
		set_speed(&motor,speed1,speed2);
	}
	
	else if (angle == 0)
	{
		set_speed(&motor,speed,speed);
	}
	
	else if (angle > 0)// turn left
	{
		speed2 = speed;
		speed1 = speed*coeffR1[angle]/100;
		set_speed(&motor,speed1,speed2);
	}
	
}

bool check_crossline(){// HAM KIEM TRA GOC 90
	rgb_setcolor(&rgb, OFF);
	bool ret = false; 
	if(
			sensor_mask(get_sensor_mask(&line_sensor), 0xe7) == 0xe7 || //1110 0111
	    sensor_mask(get_sensor_mask(&line_sensor), 0xc3)    == 0xc3   //1100 0011
		|| 	sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0xff ||
			sensor_mask(get_sensor_mask(&line_sensor), 0x81) == 0x81
	)
	{
		ret = 1;
		rgb_setcolor(&rgb, RED);
	}
	return ret;
}	

bool check_rightline(){// HAM KIEM TRA CHUYEN LAN PHAI
		rgb_setcolor(&rgb, OFF);
		bool ret = 0;
		if (check_crossline() == false) {
	  if(sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0x1f //00011111
	   ||sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0x0f  //00001111 - 0D
		||sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0x3f  //00001111 - 0D
		||sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0x7f  //00001111 - 0D
		)
		{
			
			ret = 1;
			rgb_setcolor(&rgb, GREEN);
		}
	}
	return ret;
	
}

bool check_leftline(){// HAM KIEM TRA CHUYEN LAN TRAI
	bool ret = 0;
	if (check_crossline() == false) {
	if(sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0xf8		//11111000
	 ||sensor_mask(get_sensor_mask(&line_sensor), MASK4_4) == 0xf0		//11110000
	)
	{
		ret = 1;
		rgb_setcolor(&rgb, BLUE);
	}
	}
	return ret;
}



uint8_t debounce_check(uint8_t sensor){
	uint8_t ret = 100;
	if( sensor_mask(sensor, MASK4_4)    == 0xe7 ||      //1110 0111
//	    sensor_mask(sensor, 0xc3)       == 0xc3 ||   //1100 0011
			sensor_mask(sensor, MASK4_4)    == 0xff    //1111 1111
//			sensor_mask(sensor, MASK4_4)    == 0x81      //1000 0001
	   )
		{
			ret = 0;
		  rgb_setcolor(&rgb, BLUE);
		}
		
	else	if(sensor_mask(sensor, 0xf8) == 0xf8		//11111000
//	 ||sensor_mask(sensor, MASK4_4) == 0xf0		//11110000
// 	 ||sensor_mask(sensor, MASK4_4) == 0xfc		//11111100
//   ||sensor_mask(sensor, MASK4_4) == 0xfe    //11111110
	 )
		{
			ret = 1;
   	  rgb_setcolor(&rgb, RED);
		}
	else if(
		  sensor_mask(sensor, 0x1f) == 0x1f //00011111
//	   ||sensor_mask(sensor, MASK4_4) == 0x0f  //00001111 - 0D
// 	   ||sensor_mask(sensor, MASK4_4) == 0x3f  //00111111 - 07
// 	   ||sensor_mask(sensor, MASK4_4) == 0x7f  //01111111
			)
			{
				ret = 2;
			  rgb_setcolor(&rgb, GREEN);
			}
	else ret = 100;
	return ret;
}

uint8_t sensor_mask(uint8_t sensor, uint8_t MASK){// HAM MAT NA CHO CAM BIEN
	return ( sensor & MASK);
}


