/* OS-specific pieces (kept apart from the CMSIS headers: windows.h clashes with __IO/__O). */
#include <stddef.h>
#include <stdlib.h>
#ifdef _WIN32
#include <windows.h>
static LPVOID fiber_main, fiber_co;
static void (*co_fn)(void);
static VOID CALLBACK fiber_proc(LPVOID p) { (void)p; co_fn(); }
static int map_one(unsigned long a, size_t s) {
  MEMORY_BASIC_INFORMATION mi;   /* already mapped by an earlier reset of the simulator? reuse it */
  if (VirtualQuery((void *)(size_t)a, &mi, sizeof mi) && mi.State == MEM_COMMIT &&
      (size_t)mi.BaseAddress == a && mi.RegionSize >= s)
    return 0;
  return VirtualAlloc((void *)(size_t)a, s, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE) == (void *)(size_t)a ? 0 : -1;
}
int os_map(unsigned long a, size_t sa, unsigned long b, size_t sb) {
  return (map_one(a, sa) == 0 && map_one(b, sb) == 0) ? 0 : -1;
}
void os_co_create(void (*fn)(void)) {
  co_fn = fn;
  fiber_main = IsThreadAFiber() ? GetCurrentFiber() : ConvertThreadToFiber(NULL);
  fiber_co = CreateFiber(512 * 1024, fiber_proc, NULL);
}
void os_co_enter(void) { SwitchToFiber(fiber_co); }
void os_co_leave(void) { SwitchToFiber(fiber_main); }
#else
#include <sys/mman.h>
#include <ucontext.h>
static ucontext_t ctx_main, ctx_co;
int os_map(unsigned long a, size_t sa, unsigned long b, size_t sb) {
  void *x = mmap((void *)a, sa, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
  void *y = mmap((void *)b, sb, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
  return (x == MAP_FAILED || y == MAP_FAILED) ? -1 : 0;
}
void os_co_create(void (*fn)(void)) {
  static char *stack;
  if (!stack) stack = malloc(512 * 1024);
  getcontext(&ctx_co);
  ctx_co.uc_stack.ss_sp = stack; ctx_co.uc_stack.ss_size = 512 * 1024;
  ctx_co.uc_link = &ctx_main;
  makecontext(&ctx_co, fn, 0);
}
void os_co_enter(void) { swapcontext(&ctx_main, &ctx_co); }
void os_co_leave(void) { swapcontext(&ctx_co, &ctx_main); }
#endif
