/* PC "hardware" for the user's STM32 code: HAL shim, simulated clock, interrupts, memory-mapped
 * peripherals and a coroutine that runs the user's own main(). Everything under the user's
 * src/ is compiled as-is; only ST's vendor HAL *.c files are replaced by this file. */
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern int os_map(unsigned long a, size_t sa, unsigned long b, size_t sb);
extern void os_co_create(void (*fn)(void));
extern void os_co_enter(void);
extern void os_co_leave(void);

#ifdef _WIN32
#define SIM_EXPORT __declspec(dllexport)
#else
#define SIM_EXPORT __attribute__((visibility("default")))
#endif

#define PERIPH_BASE_ADDR 0x40000000UL
#define PERIPH_SIZE      0x30000UL
#define FLASH_BASE_ADDR  0x08000000UL
#define FLASH_SIZE_BYTES 0x20000UL

extern int sim_call_user_main(void);
extern void sim_glue_main(void);
extern void SysTick_Handler(void);
extern void TIM1_UP_IRQHandler(void);
extern TIM_HandleTypeDef htim1;
extern __IO uint32_t uwTick;
__IO uint32_t uwTick;
uint32_t uwTickPrio = 0;
HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_DEFAULT;
uint32_t SystemCoreClock_unused;

/* ------------------------------------------------------------------ simulated machine */
typedef struct {
  uint64_t now_us, target_us;
  uint64_t next_systick, tim_next;
  int in_isr, finished, glue, glue_started;
  uint16_t adc[8];
  uint16_t *adc_buf;
  uint32_t adc_n;
  uint32_t loop_us;
  uint32_t call_cost_x10;       /* cost of one HAL call, 0.1 us */
  int mapped;
  char err[200];
} Sim;
static Sim S;

static void co_switch_out(void) { os_co_leave(); }

static void co_entry(void) {
  sim_call_user_main();
  S.finished = 1;
  for (;;) co_switch_out();
}

/* ------------------------------------------------------------------ time and interrupts */
static uint32_t tim1_period_us(void) {
  uint64_t psc = TIM1->PSC, arr = TIM1->ARR;
  uint64_t us = (psc + 1) * (arr + 1) / 72;   /* TIM1 on 72 MHz */
  return us < 20 ? 20 : (uint32_t)us;
}
static int tim1_running(void) {
  return (TIM1->CR1 & TIM_CR1_CEN) && (TIM1->DIER & TIM_DIER_UIE);
}
static void dispatch_events(void) {
  if (S.in_isr) return;
  S.in_isr = 1;
  int guard = 0;
  while (S.now_us >= S.next_systick && guard++ < 1000) { S.next_systick += 1000; SysTick_Handler(); }
  guard = 0;
  while (tim1_running() && S.now_us >= S.tim_next && guard++ < 1000) {
    S.tim_next += tim1_period_us();
    TIM1->SR |= TIM_SR_UIF;
    TIM1_UP_IRQHandler();
  }
  if (!tim1_running()) S.tim_next = S.now_us + tim1_period_us();
  S.in_isr = 0;
}
static void burn(uint32_t us) {
  S.now_us += us;
  dispatch_events();
  if (!S.in_isr && S.now_us >= S.target_us) co_switch_out();
}
static void call_cost(void) {
  static uint32_t acc;
  acc += S.call_cost_x10;
  if (acc >= 10) { burn(acc / 10); acc %= 10; }
}
void sim_burn_loop(void) { burn(S.loop_us); }

HAL_StatusTypeDef HAL_Init(void) { return HAL_OK; }
void HAL_IncTick(void) { uwTick += uwTickFreq; }
uint32_t HAL_GetTick(void) { call_cost(); return uwTick; }
uint32_t HAL_GetTickPrio(void) { return 0; }
void HAL_Delay(uint32_t Delay) {
  if (S.glue && !S.glue_started) {   /* user main.c has no robot loop: switch to the glue loop */
    S.glue_started = 1; sim_glue_main();
  }
  uint32_t start = uwTick, wait = Delay + (uint32_t)uwTickFreq;
  while ((uint32_t)(uwTick - start) < wait) burn(50);
}
void HAL_NVIC_SetPriority(IRQn_Type a, uint32_t b, uint32_t c) { (void)a; (void)b; (void)c; }
void HAL_NVIC_EnableIRQ(IRQn_Type a) { (void)a; }
void HAL_NVIC_DisableIRQ(IRQn_Type a) { (void)a; }
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *o) { (void)o; return HAL_OK; }
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *c, uint32_t l) { (void)c; (void)l; return HAL_OK; }
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *p) { (void)p; return HAL_OK; }

/* ------------------------------------------------------------------ GPIO */
void HAL_GPIO_Init(GPIO_TypeDef *g, GPIO_InitTypeDef *i) { (void)g; (void)i; }
void HAL_GPIO_DeInit(GPIO_TypeDef *g, uint32_t p) { (void)g; (void)p; }
void HAL_GPIO_WritePin(GPIO_TypeDef *g, uint16_t p, GPIO_PinState s) {
  call_cost();
  if (s) g->ODR |= p; else g->ODR &= ~(uint32_t)p;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *g, uint16_t p) {
  call_cost();
  return (g->IDR & p) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void HAL_GPIO_TogglePin(GPIO_TypeDef *g, uint16_t p) { g->ODR ^= p; }
void HAL_GPIO_EXTI_IRQHandler(uint16_t p) { (void)p; }
__attribute__((weak)) void HAL_GPIO_EXTI_Callback(uint16_t p) { (void)p; }

/* ------------------------------------------------------------------ ADC / DMA */
__attribute__((weak)) void HAL_ADC_MspInit(ADC_HandleTypeDef *h) { (void)h; }
__attribute__((weak)) void HAL_ADC_MspDeInit(ADC_HandleTypeDef *h) { (void)h; }
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h) { HAL_ADC_MspInit(h); return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c) { (void)h; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *pData, uint32_t n) {
  (void)h;
  S.adc_buf = (uint16_t *)pData; S.adc_n = n;
  for (uint32_t i = 0; i < n && i < 8; i++) S.adc_buf[i] = S.adc[i];
  return HAL_OK;
}
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *h) { (void)h; return HAL_OK; }
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *h) { (void)h; }

/* ------------------------------------------------------------------ timers */
__attribute__((weak)) void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *h) { (void)h; }
__attribute__((weak)) void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *h) { (void)h; }
__attribute__((weak)) void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *h) { (void)h; }
__attribute__((weak)) void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *h) { (void)h; }
__attribute__((weak)) void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *h) { (void)h; }
static void tim_load(TIM_HandleTypeDef *h) {
  h->Instance->PSC = h->Init.Prescaler;
  h->Instance->ARR = h->Init.Period;
}
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *h) { HAL_TIM_Base_MspInit(h); tim_load(h); return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *h) { HAL_TIM_PWM_MspInit(h); tim_load(h); return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *h, const TIM_ClockConfigTypeDef *c) { (void)h; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *h, const TIM_MasterConfigTypeDef *c) { (void)h; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *h, const TIM_OC_InitTypeDef *c, uint32_t ch) {
  switch (ch) {
    case TIM_CHANNEL_1: h->Instance->CCR1 = c->Pulse; break;
    case TIM_CHANNEL_2: h->Instance->CCR2 = c->Pulse; break;
    case TIM_CHANNEL_3: h->Instance->CCR3 = c->Pulse; break;
    default:            h->Instance->CCR4 = c->Pulse; break;
  }
  return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch) {
  h->Instance->CCER |= (TIM_CCER_CC1E << ch);
  h->Instance->CR1 |= TIM_CR1_CEN;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *h) { h->Instance->CR1 |= TIM_CR1_CEN; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *h) { h->Instance->CR1 &= ~TIM_CR1_CEN; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h) {
  h->Instance->DIER |= TIM_DIER_UIE;
  h->Instance->CR1 |= TIM_CR1_CEN;
  if (h->Instance == TIM1) S.tim_next = S.now_us + tim1_period_us();
  return HAL_OK;
}
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *h) {
  if (h->Instance->SR & TIM_SR_UIF) {
    h->Instance->SR &= ~TIM_SR_UIF;
    HAL_TIM_PeriodElapsedCallback(h);
  }
}

/* ------------------------------------------------------------------ flash */
static int flash_unlocked;
HAL_StatusTypeDef HAL_FLASH_Unlock(void) { flash_unlocked = 1; return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { flash_unlocked = 0; return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint64_t data) {
  if (!flash_unlocked || addr < FLASH_BASE_ADDR || addr >= FLASH_BASE_ADDR + FLASH_SIZE_BYTES) return HAL_ERROR;
  uint16_t *p = (uint16_t *)(uintptr_t)addr;
  if (type == FLASH_TYPEPROGRAM_HALFWORD) *p &= (uint16_t)data;
  else if (type == FLASH_TYPEPROGRAM_WORD) { p[0] &= (uint16_t)data; p[1] &= (uint16_t)(data >> 16); }
  return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *err) {
  if (!flash_unlocked) return HAL_ERROR;
  if (e->TypeErase == FLASH_TYPEERASE_PAGES) {
    for (uint32_t i = 0; i < e->NbPages; i++) {
      uint32_t a = e->PageAddress + i * 1024;
      if (a >= FLASH_BASE_ADDR && a < FLASH_BASE_ADDR + FLASH_SIZE_BYTES) memset((void *)(uintptr_t)a, 0xFF, 1024);
    }
  }
  if (err) *err = 0xFFFFFFFFU;
  return HAL_OK;
}

/* ------------------------------------------------------------------ memory map */
static int map_hw(void) { return os_map(PERIPH_BASE_ADDR, PERIPH_SIZE, FLASH_BASE_ADDR, FLASH_SIZE_BYTES); }

/* ------------------------------------------------------------------ exported API */
typedef struct {
  int32_t pwm_l, pwm_r, arr;
  int32_t dir_l, dir_r, brake_l, brake_r;
  int32_t servo_ccr, servo_center, servo_arr;
  int32_t runcase, running, line_lost, cnt1;
  uint32_t odr_a, odr_b, odr_c;
  uint32_t tick_ms, tim1_us;
  int32_t robot_ready;   /* robot_init() has run */
  int32_t finished;
} SimOut;

extern void sim_fill_out(SimOut *o);
extern void sim_apply_inputs(uint8_t btn_mask, uint8_t dip);

SIM_EXPORT const char *sim_error(void) { return S.err; }

SIM_EXPORT int sim_start(int glue, uint32_t loop_us, uint32_t call_cost_x10, const uint16_t *flash_init) {
  (void)flash_init;
  memset(&S, 0, sizeof S);
  if (map_hw() != 0) {
    snprintf(S.err, sizeof S.err, "cannot map peripherals at 0x40000000 / flash at 0x08000000");
    return -1;
  }
  memset((void *)PERIPH_BASE_ADDR, 0, PERIPH_SIZE);          /* power-on register state */
  memset((void *)FLASH_BASE_ADDR, 0xFF, FLASH_SIZE_BYTES);
  GPIOA->IDR = GPIOB->IDR = GPIOC->IDR = GPIOD->IDR = 0xFFFF;   /* pull-ups */
  uwTick = 0;
  S.glue = glue; S.loop_us = loop_us ? loop_us : 50; S.call_cost_x10 = call_cost_x10;
  S.next_systick = 1000;
  os_co_create(co_entry);
  return 0;
}

SIM_EXPORT void sim_set_adc(const uint16_t *v) {
  memcpy(S.adc, v, sizeof S.adc);
  if (S.adc_buf) for (uint32_t i = 0; i < S.adc_n && i < 8; i++) S.adc_buf[i] = v[i];
}
SIM_EXPORT void sim_inputs(uint8_t btn_mask, uint8_t dip) { sim_apply_inputs(btn_mask, dip); }

SIM_EXPORT void sim_run(uint32_t us, SimOut *out) {
  S.target_us = S.now_us + us;
  if (!S.finished) {
    os_co_enter();
  }
  if (S.finished) S.now_us = S.target_us;
  sim_fill_out(out);
  out->tick_ms = uwTick; out->tim1_us = tim1_period_us(); out->finished = S.finished;
}
