/* Glue between the PC simulator and the user's own objects. It only READS the user's globals
 * (motor, servo, buttons, DIP switches, MCR_robot) to learn which pins/channels they used. */
#include "main.h"
#include "robot.h"
#include "motor.h"
#include "servo.h"
#include "button.h"
#include "dip_switch.h"

extern Robot MCR_robot;
extern TIM_HandleTypeDef *htim[];
extern Motor motor;
extern Servo servo;
extern Button button1, button2, button3;
extern dip_switch dip1, dip2, dip3;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern void sim_burn_loop(void);

static int robot_ready;

/* Same two lines that are commented out in the user's main.c, then the main loop. */
void sim_glue_main(void) {
  robot_init(&MCR_robot, &hadc1, &hdma_adc1, htim);
  HAL_TIM_Base_Start_IT(&htim1);
  robot_ready = 1;
  for (;;) { loop(&MCR_robot); sim_burn_loop(); }
}

static void set_pin(GPIO_TypeDef *g, uint16_t pin, int low) {
  if (!g) return;
  if (low) g->IDR &= ~(uint32_t)pin; else g->IDR |= pin;
}

void sim_apply_inputs(uint8_t btn_mask, uint8_t dip) {
  Button *b[3] = {&button1, &button2, &button3};
  dip_switch *d[3] = {&dip1, &dip2, &dip3};
  for (int i = 0; i < 3; i++) set_pin(b[i]->GPIOx, b[i]->GPIO_Pin, (btn_mask >> i) & 1);   /* active low */
  for (int i = 0; i < 3; i++) set_pin(d[i]->GPIOx, d[i]->GPIO_Pin, (dip >> (2 - i)) & 1);   /* dip1 = MSB */
}

typedef struct {
  int32_t pwm_l, pwm_r, arr;
  int32_t dir_l, dir_r, brake_l, brake_r;
  int32_t servo_ccr, servo_center, servo_arr;
  int32_t runcase, running, line_lost, cnt1;
  uint32_t odr_a, odr_b, odr_c;
  uint32_t tick_ms, tim1_us;
  int32_t robot_ready;
  int32_t finished;
} SimOut;

static uint32_t ccr(TIM_TypeDef *t, uint16_t ch) {
  switch (ch) {
    case TIM_CHANNEL_1: return t->CCR1;
    case TIM_CHANNEL_2: return t->CCR2;
    case TIM_CHANNEL_3: return t->CCR3;
    default:            return t->CCR4;
  }
}
static int pin(GPIO_TypeDef *g, uint16_t p) { return g ? ((g->ODR & p) != 0) : 0; }

void sim_fill_out(SimOut *o) {
  o->robot_ready = robot_ready || (motor.htim != 0);
  if (motor.htim) {
    TIM_TypeDef *t = motor.htim->Instance;
    int on_l = (t->CCER >> motor.left_motor_channel) & 1, on_r = (t->CCER >> motor.right_motor_channel) & 1;
    o->pwm_l = on_l ? ccr(t, motor.left_motor_channel) : 0;
    o->pwm_r = on_r ? ccr(t, motor.right_motor_channel) : 0;
    o->arr = t->ARR;
    o->dir_l = pin(motor.GPIO_DIR_PORT, motor.dir_left_pin);
    o->dir_r = pin(motor.GPIO_DIR_PORT, motor.dir_right_pin);
    o->brake_l = pin(motor.GPIO_BRK_PORT, motor.brake_left_pin);
    o->brake_r = pin(motor.GPIO_BRK_PORT, motor.brake_right_pin);
  }
  if (servo.htim) {
    o->servo_ccr = ccr(servo.htim->Instance, servo.channel);
    o->servo_center = servo.servo_center;
    o->servo_arr = servo.htim->Instance->ARR;
  }
  o->runcase = MCR_robot.runcase; o->running = MCR_robot.running;
  o->line_lost = MCR_robot.line_lost; o->cnt1 = MCR_robot.cnt1;
  o->odr_a = GPIOA->ODR; o->odr_b = GPIOB->ODR; o->odr_c = GPIOC->ODR;
}

#ifdef _WIN32
#define SIM_EXPORT __declspec(dllexport)
#else
#define SIM_EXPORT __attribute__((visibility("default")))
#endif
/* debug: user's own view of DIP / buttons (state flags) */
SIM_EXPORT int sim_debug(void) {
  dip_switch *d[3] = {&dip1, &dip2, &dip3};
  return get_button(&button1) | (get_button(&button2) << 1) | (get_button(&button3) << 2) | (get_sw_mask(d, 3) << 4);
}
