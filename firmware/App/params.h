#ifndef PARAMS_H
#define PARAMS_H
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define         MASK2_2         0x66    /* xooxxoox   01100110                  */
#define         MASK2_0         0xc0    /* ooxxxxxx   01100000                  */
#define         MASK0_2         0x03    /* xxxxxxoo   00000110                  */
#define         MASK3_3         0xe7    /* oooxxooo   11100111                  */
#define         MASK0_3         0x07    /* xxxxxooo   00000111                  */
#define         MASK3_0         0xe0    /* oooxxxxx   11100000                  */
#define         MASK4_0         0xf0    /* ooooxxxx   11110000                  */
#define         MASK0_4         0x0f    /* xxxxoooo   00001111                  */
#define         MASK1_1         0x81    /* oxxxxxxo   01111110                  */
#define					MASK4_4					0xff    /* oooooooo	  11111111									*/

#define ROBOT_SPEED 60 // TOC DO CUA XE
#define LINE_GAP_BRIDGE_MS 100U

uint16_t calib_weight[8] = {500,500,500,650,700,500,500,500}; // HIEU CHINH GIA TRI CHO TUNG CAM BIEN

/////----------- Differential values -------------------//////
const char coeffR1[50] ={
	100,	98, 	96, 	94, 	93, 	91, 	90, 	88, 	86, 	85,
	83, 	82, 	80, 	79, 	77, 	76, 	74, 	73, 	72, 	70,
	69, 	67, 	66, 	64, 	63, 	62, 	60, 	59, 	58, 	56,
	55, 	53, 	52, 	50, 	49, 	48, 	46, 	45, 	43, 	42,
	40, 	39, 	37, 	36, 	34, 	33, 	31, 	30, 	28, 	27
};
// TINH VI SAI


#endif
