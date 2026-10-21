#include "calib.h" 

void calib_servo(Servo *servo, Button* buttons[], RGB *rgb,Buzzer *buzz){ // HAM HIEU CHINH SERVO
	
	rgb_setcolor(rgb, GREEN);
	if(get_button(buttons[0]))
	{
		HAL_Delay(100);
		update_servo_center(servo, +1);
		handle(servo,0);
	}
	
	if(get_button(buttons[1]))
	{
		HAL_Delay(100);
		update_servo_center(servo, -1);
		handle(servo,0);
	}
	if(get_button(buttons[2]))
	{
		save_servo_center(servo);
		HAL_Delay(100);
		beep_long(buzz, 100);
	}
}

void calib_sensor(ADC* line_sensor, Button* buttons[], RGB *rgb, Buzzer *buzz){ // HAM HIEU CHINH CAM BIEN
	rgb_setcolor(rgb, RED);
	if(get_button(buttons[0]))
	{	
		HAL_Delay(100);	
		update_black(line_sensor);
		beep_long(buzz, 100);	
	}
	if (get_button(buttons[1]))
	{
		HAL_Delay(100);
		update_white(line_sensor);
		beep_long(buzz, 100);	
	}		
}

void write_read_sensor_value(ADC* line_sensor,Button* buttons[], RGB *rgb,Buzzer *buzz){ // HAM LUU CAM BIEN
	rgb_setcolor(rgb, BLUE);
	if(get_button(buttons[0]))
	{
		HAL_Delay(100);
		write_vcompare(line_sensor);
		beep_long(buzz, 100);	
	}
	if(get_button(buttons[1]))
	{
		HAL_Delay(100);
		read_vcompare(line_sensor);
		beep_long(buzz, 100);	
	}
}

void motor_check(Motor* motor,Button* buttons[], RGB *rgb){ // HAM KIEM TRA DONG CO
	rgb_setcolor(rgb, GREEN);
	if (get_button(buttons[0]))   // press SW2 to run left motor backward
	{
		
		set_speed(motor,50,50);
		//HAL_Delay(10);
		
	}
	 if(get_button(buttons[1]))// press SW1 to run left motor forward
	{
		
		set_speed(motor,-50,-50);;
	//	HAL_Delay(10);
	}
	if (get_button(buttons[2])){		
		set_brake(motor,1,1);
		set_speed(motor,0,0);
		HAL_Delay(200);
		set_brake(motor,0,0);
		HAL_Delay(200);		
	}	// stop
	
}	

void check_sonic(Sonic *sonic, RGB *rgb,Buzzer *buzz){ // HAM KIEM TRA SONIC
		rgb_setcolor(rgb, OFF);
		float dist = sonic_dist(sonic);
		if(dist > 30.0)
		{
			rgb_setcolor(rgb, WHITE);
			HAL_Delay(250);
			beep_long(buzz, 100);	
			rgb_setcolor(rgb, OFF);
			HAL_Delay(250);
		}	
}


RunState  run_with_gate_bar(Sonic *sonic,RGB * rgb, uint8_t SW, bool ready, bool run){		// HAM CHAY VOI THANH CHAN VA ULTRASONIC
		RunState state;
		state.ready = ready;
    state.run = run;
	
		if (SW) state.ready =~ state.ready;
		if (ready){
			rgb_setcolor(rgb, WHITE);
			float dist = sonic_dist(sonic);
			if( dist > 30.0)
			{
				state.run = true;
			}	
		}
		else{
			float dist = sonic_dist(sonic);
			rgb_setcolor(rgb, RED);
			if( dist < 30.0){
			rgb_setcolor(rgb, GREEN);
			state.run = false;
			}	
		}
	return state;
}
	
//END




