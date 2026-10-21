/* Compiles the user's main.c UNMODIFIED; only its entry point is renamed so it can be started
 * as a coroutine. SIM_MAIN_FILE is the path of main.c (given with -D by the build script). */
#define main sim_user_main
#include SIM_MAIN_FILE
#undef main
int sim_call_user_main(void) { return sim_user_main(); }
