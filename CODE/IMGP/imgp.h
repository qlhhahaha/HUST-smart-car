/*
 * imgp.h
 *
 *  Created on: 2021年5月20日
 *      Author: 朱江禹
 */

#ifndef _IMGP_H_
#define _IMGP_H_

#include "common.h"
#include "SEEKFREE_MT9V03X.h"
#include "motor.h"

#define FIXED_THRES 160
#define f(M, I, J, W) (*((M) + (I) * (W) + (J)))
#define abs(x, y) (((x) < 0 ? (-(x)) : (x)) + ((y) < 0 ? (-(y)) : (y)))
#define ABS(X) ((X) >= 0 ? (X) : (-X))
#define ran_u8(x) ((x) > 255 ? 255 : (x))
#define LINE_GET 7
#define LINE_LOST 4
#define CEN_MIN 32.5
#define CEN_MAX 38.5
#define ONEE_MAX 1.5
#define ONEE_MIN -1.5
#define TRI_THSD 18
#define F_R_ERROR -29.5
#define F_L_ERROR 29.5
#define F_COUNTS 13
#define C_COUNTS 35

#define STOP_POINT_THRS 23

#define LEAVE_JUMP_COUNT 10
#define ENTER_JUMP_COUNT 7

#define LEAVE_GARAGE_TURN_COUNT 12
#define LEAVE_GARAGE_STRAIGHT_COUNT 10

#define ENTER_GARAGE_COUNT 15

#define LEAVE_GARAGE_ERROR -29
#define ENTER_GARAGE_ERROR -29

#define ENTER_ROUND_STRAIGHT_COUNT 10
#define ENTER_ROUND_TURN_COUNT 15
#define LEAVE_ROUND_COUNT 15

#define RIGHT_ENTER_ROUND_ERROR -23.5
#define RIGHT_LEAVE_ROUND_ERROR -30

#define LEFT_ENTER_ROUND_ERROR 23.5
#define LEFT_LEAVE_ROUND_ERROR 30

#define LEAVE_ROUND_CHECK_LINE 5 //8个点中至少得有6个点满足

#define FIRST_ZEBRA_COUNT 10

void sobel_init(uint8 *simg, uint16 width, uint16 height);
void id_init(ImgData *pd_last, ImgData *pd_now);
void img_get(uint8 *src, uint8 *dst, uint16 src_w, uint16 src_h);
void sobel_get(uint8 *src, uint8 *dst, uint16 width, uint16 height);
void binary(uint8 *img, uint16 width, uint16 height, uint8 threshold);
void binary2(uint8 *img, uint8 *sob, uint16 width,
             uint16 height, uint8 img_ths, uint8 sob_ths);
void binary3(uint8 *img, uint8 *fixed_thres, uint16 width,
             uint16 height);
void id_reset(ImgData *pd);
void id_get(uint8 *src, ImgData **ppd_last, ImgData **ppd_now);
void id_getleft(uint8 *src, ImgData *pd);
void id_getright(uint8 *src, ImgData *pd);
void id2img(ImgData *pd, uint8 *sob, uint16 width, uint16 height);
void line_getkd(Line *pl, uint16 i, uint16 j);
void line_qfit(ImgData *pd, Line *pl);

void leave_zebra(uint8 *src, uint16 i, uint16 j);
void enter_zebra(uint8 *src, uint16 i, uint16 j, ImgData **ppd_last, ImgData **ppd_now);
void check_stop(uint8 *src, uint16 i, uint16 j);

void round_check(int16 *value);

void fork_check(uint8 *src, Line *pl, Line *pr, uint16 lenl1, uint16 lenr1);
void fork_dianci_check(int16 *value);

#endif
