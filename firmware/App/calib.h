#ifndef CALIB_H
#define CALIB_H
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include "adc.h"
#include "motor.h"
#include "servo.h"
#include "buzzer.h"
#include "rgb.h"
#include "button.h"
#include "adc.h"
#include "sonic.h"


void calib_servo(Servo *servo, Button* buttons[], RGB *rgb,Buzzer *buzz);
void calib_sensor(ADC* line_sensor,Button* buttons[],RGB *rgb,Buzzer *buzz);
void write_read_sensor_value(ADC* line_sensor,Button* buttons[], RGB *rgb,Buzzer *buzz);
void motor_check(Motor* motor,Button* buttons[], RGB *rgb);
void check_sonic(Sonic *sonic, RGB *rgb,Buzzer *buzz);

typedef struct {
    bool ready;
    bool run;
} RunState;

RunState run_with_gate_bar(Sonic *sonic,RGB * rgb, uint8_t SW, bool ready, bool run);

#endif
