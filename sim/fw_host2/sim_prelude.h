/* Force-included before every user source. Neutralises the ARM inline assembly of CMSIS
 * (cmsis_gcc.h) so the REAL CMSIS/HAL headers can be parsed on a PC. No user file is touched. */
#ifndef SIM_PRELUDE_H
#define SIM_PRELUDE_H
#include <stdint.h>
#define __CMSIS_GCC_H
#define __ASM __asm
#define __INLINE inline
#define __STATIC_INLINE static inline
#define __STATIC_FORCEINLINE static inline
#define __NO_RETURN __attribute__((__noreturn__))
#define __USED __attribute__((used))
#define __WEAK __attribute__((weak))
#define __PACKED __attribute__((packed, aligned(1)))
#define __PACKED_STRUCT struct __attribute__((packed, aligned(1)))
#define __PACKED_UNION union __attribute__((packed, aligned(1)))
#define __ALIGNED(x) __attribute__((aligned(x)))
#define __RESTRICT __restrict
#define __COMPILER_BARRIER() ((void)0)
#define __NOP() ((void)0)
#define __WFI() ((void)0)
#define __WFE() ((void)0)
#define __SEV() ((void)0)
#define __ISB() ((void)0)
#define __DSB() ((void)0)
#define __DMB() ((void)0)
static inline void __enable_irq(void) {}
static inline void __disable_irq(void) {}
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __set_PRIMASK(uint32_t v) { (void)v; }
static inline uint32_t __get_BASEPRI(void) { return 0; }
static inline void __set_BASEPRI(uint32_t v) { (void)v; }
static inline uint32_t __get_FAULTMASK(void) { return 0; }
static inline void __set_FAULTMASK(uint32_t v) { (void)v; }
static inline uint32_t __get_CONTROL(void) { return 0; }
static inline void __set_CONTROL(uint32_t v) { (void)v; }
static inline uint32_t __get_IPSR(void) { return 0; }
static inline uint32_t __get_APSR(void) { return 0; }
static inline uint32_t __get_xPSR(void) { return 0; }
static inline uint32_t __get_PSP(void) { return 0; }
static inline void __set_PSP(uint32_t v) { (void)v; }
static inline uint32_t __get_MSP(void) { return 0; }
static inline void __set_MSP(uint32_t v) { (void)v; }
static inline uint32_t __REV(uint32_t v) { return __builtin_bswap32(v); }
static inline uint32_t __RBIT(uint32_t v) { uint32_t r = 0; for (int i = 0; i < 32; i++) { r = (r << 1) | (v & 1); v >>= 1; } return r; }
static inline uint8_t __CLZ(uint32_t v) { return v ? (uint8_t)__builtin_clz(v) : 32; }
#endif
