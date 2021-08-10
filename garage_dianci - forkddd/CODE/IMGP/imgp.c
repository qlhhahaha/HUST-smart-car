/*
 * imgp.c
 *
 *  Created on: 2021年5月20日
 *      Author: 朱江禹
 */

#include "imgp.h"
#include "zf_stm_systick.h"
const int16 sobel_x[3][3] =
    {{-1, 0, 1},
     {-2, 0, 2},
     {-1, 0, 1}};
const int16 sobel_y[3][3] =
    {{-1, -2, -1},
     {0, 0, 0},
     {1, 2, 1}};
const int16 pointi = IMG_H - 2;
const int16 pointj = IMG_W / 2;

uint8 img[IMG_H][IMG_W];
uint8 sobel[IMG_H][IMG_W];

uint8 fixed_thres[IMG_H][IMG_W];

float32 sobel_thsf; //待确定阈值
float32 sobel_k = 2.25;
float32 img_thsf;
float32 img_k = 0.8;
float32 error[5];
int16 col_l, col_r;

int16 enter_jump_count;
int16 leave_jump_count;
int16 enter_gar_time;
int16 stop_flag;
int16 pwm_stop_flag;
int16 turn_speed_flag;
int16 fork_check_flag;
int16 fork_check_flag_1;

int16 l_angle_flag;
int16 r_angle_flag;

int16 fixed_thres_value = 160;

int16 max_col;

int16 right_enter_round_flag;
int16 left_enter_round_flag;

int16 right_leave_round_point;
int16 left_leave_round_point;
int16 right_leave_round_flag;
int16 left_leave_round_flag;

int16 dianci_enter_fork_flag;
int16 dianci_right_leave_fork_flag;
int16 dianci_left_leave_fork_flag;

extern int16 induc_value[5];

ImgData id[2];
ImgData *pd_last, *pd_now;

void sobel_init(uint8 *simg, uint16 width, uint16 height)
{
    int16 i, j1, j2;
    j1 = 0;
    j2 = height - 1;
    for (i = 0; i < width; i++)
    {
        f(simg, j1, i, width) =
            f(simg, j2, i, width) = 255;
    }
    j2 = width - 1;
    for (i = 0; i < height; i++)
    {
        f(simg, i, j1, width) =
            f(simg, i, j2, width) = 255;
    }
}

void id_init(ImgData *pd_last, ImgData *pd_now)
{
    int16 i;
    pd_last->state =
        pd_now->state = ID_LEAVE_GAR;
    for (i = 0; i < LINE_LEN; i++)
    {
        pd_last->l1.pos[i] =
            pd_now->l1.pos[i] = 1;
        pd_last->r1.pos[i] =
            pd_now->r1.pos[i] = IMG_W - 2;
    }
    pd_last->l1.exist =
        pd_last->r1.exist =
            pd_now->l1.exist =
                pd_now->r1.exist = 0;
    error[0] =
        error[1] =
            error[2] =
                error[3] =
                    error[4] = 0.0;
    pd_last->tri_w[0] = pd_last->tri_w[7] =
        pd_now->tri_w[0] = pd_now->tri_w[7] = -1.0;
    pd_last->tri_w[1] = pd_last->tri_w[6] =
        pd_now->tri_w[1] = pd_now->tri_w[6] = 0.5;
    pd_last->tri_w[2] = pd_last->tri_w[5] =
        pd_now->tri_w[2] = pd_now->tri_w[5] = 1.0;
    pd_last->tri_w[3] = pd_last->tri_w[4] =
        pd_now->tri_w[3] = pd_now->tri_w[4] = 1.5;
    pd_last->f_time = pd_now->f_time = 0;
    pd_last->f_dir = pd_now->f_dir = F_DIR_R;
    pd_last->f_count = pd_now->f_count = 0;
    pd_last->c_count = pd_now->c_count = 0;
    pd_last->l_g_count = pd_now->l_g_count = 0;
    pd_last->e_g_count = pd_now->e_g_count = 0;

    pd_last->enter_straight_count = pd_now->enter_straight_count = 0;
    pd_last->enter_turn_count = pd_now->enter_turn_count = 0;
    pd_last->leave_turn_count = pd_now->leave_turn_count = 0;
    pd_last->first_zebra_count = pd_now->first_zebra_count = 0;
    pd_last->leave_garage_straight_count = pd_now->leave_garage_straight_count = 0;
}

void img_get(uint8 *src, uint8 *dst, uint16 src_w, uint16 src_h)
{
    int16 i, i1, j, j1;
    uint16 dst_w, dst_h, dst_w1, sum;
    uint64 sum_all;
    dst_w = src_w / 2;
    dst_h = src_h * 2 / 3 / 2;
    dst_w1 = dst_w - 1;
    sum_all = 0ll;
    for (i = 0, i1 = IMG_LB; i < dst_h; i++, i1 += 2)
    {
        for (j = 0, j1 = 0; j < dst_w1; j++, j1 += 2)
        {
            sum = (f(src, i1, j1, src_w) + f(src, i1 + 1, j1, src_w) + f(src, i1, j1 + 1, src_w) + f(src, i1 + 1, j1 + 1, src_w) + 2) / 4;
            f(dst, i, j, dst_w) = (uint8)sum;
            sum_all += sum;
        }
    }
    j = dst_w1;
    j1 = j * 2;
    for (i = 0, i1 = IMG_LB; i < dst_h; i++, i1 += 2)
    {
        sum = (f(src, i1, j1, src_w) + f(src, i1 + 1, j1, src_w) + 1) / 2;
        f(dst, i, j, dst_w) = (uint8)sum;
        sum_all += sum;
    }
    img_thsf = (float32)sum_all;
    img_thsf /= dst_w * dst_h;
    img_thsf *= img_k;
    if (img_thsf > 253.0)
        img_thsf = 253.0;
}

void sobel_get(uint8 *src, uint8 *dst, uint16 width, uint16 height)
{
    int16 i, j, sumx, sumy, sum;
    uint64 sum_all;
    uint16 w1, h1; //边界值
    w1 = width - 1;
    h1 = height - 1;
    sum_all = 0ll;
    for (i = 1; i < h1; i++)
    {
        for (j = 1; j < w1; j++)
        {
            sumx = sobel_x[0][0] * f(src, i - 1, j - 1, width) + sobel_x[1][0] * f(src, i, j - 1, width) + sobel_x[2][0] * f(src, i + 1, j - 1, width) + sobel_x[0][2] * f(src, i - 1, j + 1, width) + sobel_x[1][2] * f(src, i, j + 1, width) + sobel_x[2][2] * f(src, i + 1, j + 1, width);
            sumy = sobel_y[0][0] * f(src, i - 1, j - 1, width) + sobel_y[0][1] * f(src, i - 1, j, width) + sobel_y[0][2] * f(src, i - 1, j + 1, width) + sobel_y[2][0] * f(src, i + 1, j - 1, width) + sobel_y[2][1] * f(src, i + 1, j, width) + sobel_y[2][2] * f(src, i + 1, j + 1, width);
            sum = ran_u8(abs(sumx, sumy));
            f(dst, i, j, width) = (uint8)sum;
            sum_all += sum;
        }
    }
    sobel_thsf = (float32)sum_all;
    sobel_thsf /= width * height;
    sobel_thsf *= sobel_k;
    if (sobel_thsf > 253.0)
        sobel_thsf = 253.0;
}

void binary(uint8 *img, uint16 width, uint16 height, uint8 threshold)
{
    int i, j;
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            f(img, i, j, width) = f(img, i, j, width) > threshold ? 255 : 0;
        }
    }
}

void binary2(uint8 *img, uint8 *sob, uint16 width,
             uint16 height, uint8 img_ths, uint8 sob_ths)
{
    int16 i, j;
    int16 h1, w1;
    h1 = height - 1;
    w1 = width - 1;
    for (i = 1; i < h1; i++)
    {
        for (j = 1; j < w1; j++)
        {
            f(sob, i, j, width) = f(sob, i, j, width) > sob_ths ? (f(img, i, j, width) > img_ths ? 0 : 255) : 0;
        }
    }
}

void binary3(uint8 *img, uint8 *fixed_thres, uint16 width,
             uint16 height)
{
    int16 i, j;
    int16 h1, w1;
    h1 = height - 1;
    w1 = width - 1;
    for (i = 1; i < h1; i++)
    {
        for (j = 1; j < w1; j++)
        {
            f(fixed_thres, i, j, width) = f(img, i, j, width) > fixed_thres_value ? 255 : 0;
        }
    }
}

void id_reset(ImgData *pd)
{
    pd->l1.exist =
        pd->r1.exist = 0;
}

void id_get(uint8 *src, ImgData **ppd_last, ImgData **ppd_now)
{
    ImgData *p0;
    Line *pl1, *pr1;
    uint16 lenl1, lenr1;
    int16 i, i1, j;
    float32 f1, f2;

    //二级指针，交换一级指针的地址，节省时间空间
    p0 = *ppd_last;
    *ppd_last = *ppd_now;
    *ppd_now = p0;

    //now的数据给到last，再把now给初始化，其中用于计数的count，time之类的全部沿用last
    id_reset(*ppd_now);
    (*ppd_now)->f_time = (*ppd_last)->f_time;
    (*ppd_now)->f_count = (*ppd_last)->f_count;
    (*ppd_now)->f_dir = (*ppd_last)->f_dir;
    (*ppd_now)->c_count = (*ppd_last)->c_count;
    (*ppd_now)->l_g_count = (*ppd_last)->l_g_count;
    (*ppd_now)->e_g_count = (*ppd_last)->e_g_count;
    (*ppd_now)->first_zebra_count = (*ppd_last)->first_zebra_count;
    (*ppd_now)->leave_garage_straight_count = (*ppd_last)->leave_garage_straight_count;

    (*ppd_now)->enter_straight_count = (*ppd_last)->enter_straight_count;
    (*ppd_now)->enter_turn_count = (*ppd_last)->enter_turn_count;
    (*ppd_now)->leave_turn_count = (*ppd_last)->leave_turn_count;

    error[0] = error[1];
    error[1] = error[2];
    error[2] = error[3];
    error[3] = error[4];
    //设四个error是为了后续使用I系数，但PD控制则只用到error[3]和error[4]

    pl1 = &((*ppd_now)->l1);
    pr1 = &((*ppd_now)->r1);

    if ((*ppd_last)->state == ID_STRAI) //直线模式
    {
        turn_speed_flag = 0;
        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);

        //得到直线的斜率和到底边中点的距离
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);

        lenl1 = pl1->begin - pl1->lost + 1; //指竖直方向上的长度
        lenr1 = pr1->begin - pr1->lost + 1;
        //斑马线和停车check
        /* leave_zebra(src, pointi, pointj);*/
        enter_zebra(src, pointi, pointj, &pd_last, &pd_now);

        fork_check(src, pl1, pr1, lenl1, lenr1);

        if ((*ppd_now)->state == ID_FIRST_ZEBRA)
            return;

        round_check(induc_value);

        /*check_stop(src, pointi, pointj);*/

        if (pl1->exist &&
            pr1->exist &&
            (lenl1 > LINE_LOST) &&
            (lenr1 > LINE_LOST) &&
            (enter_gar_time < 2) &&
            (right_enter_round_flag == 0) &&
            left_enter_round_flag == 0)
        { //左右都未丢线且不是入库不是入环
            error[4] = pl1->d - pr1->d;
            (*ppd_now)->state = ID_STRAI; //模式不变
        }

        /*else if (leave_jump_count >= LEAVE_JUMP_COUNT)
        { //竖直方向的跳变点足够多时进入出库模式
            leave_jump_count = 0;
            (*ppd_now)->l_g_count = 0;
            (*ppd_now)->state = ID_LEAVE_GAR;
        }*/

        else if (enter_gar_time >= 2)
        { //横向斑马线出现两次时进入入库模式
            enter_gar_time = 0;
            (*ppd_now)->e_g_count = 0;
            (*ppd_now)->state = ID_ENTER_GAR;
        }

        /* else if (stop_flag == 1)
        { //变为停车模式
            stop_flag = 0;
            (*ppd_now)->state = ID_STOP;
        }*/

        else if (right_enter_round_flag == 1)
        { //变为右入环模式
            right_enter_round_flag = 0;
            (*ppd_now)->enter_straight_count = 0;
            (*ppd_now)->enter_turn_count = 0;
            (*ppd_now)->state = ID_RIGHT_ENTER_ROUND;
        }

        else if (left_enter_round_flag == 1)
        { //变为左入环模式
            left_enter_round_flag = 0;
            (*ppd_now)->enter_straight_count = 0;
            (*ppd_now)->enter_turn_count = 0;
            (*ppd_now)->state = ID_LEFT_ENTER_ROUND;
        }

        else if ((!pl1->exist) || (lenl1 <= LINE_LOST))
        { //根据else-if的顺序，能进行这个if判断的话，前面几个特殊元素的条件必不满足

            //error的具体算法？
            (*ppd_now)->center = (0.5 * error[3] + (*ppd_last)->r1.d) * 0.95;
            if ((*ppd_now)->center > CEN_MAX)
                (*ppd_now)->center = CEN_MAX;
            else if ((*ppd_now)->center < CEN_MIN)
                (*ppd_now)->center = CEN_MIN;
            error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
            if (error[4] < ONEE_MIN)
                error[4] = ONEE_MIN;
            (*ppd_now)->state = ID_ONER; //变为右单线模式
        }

        else if ((!pr1->exist) || (lenr1 <= LINE_LOST))
        {
            (*ppd_now)->center = ((*ppd_last)->l1.d - 0.5 * error[3]) * 0.95;
            if ((*ppd_now)->center > CEN_MAX)
                (*ppd_now)->center = CEN_MAX;
            else if ((*ppd_now)->center < CEN_MIN)
                (*ppd_now)->center = CEN_MIN;
            error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
            if (error[4] > ONEE_MAX)
                error[4] = ONEE_MAX;
            (*ppd_now)->state = ID_ONEL; //变为左单线模式
        }

        else if (!(pl1->exist || (lenl1 <= LINE_LOST)) && !(pr1->exist || (lenr1 <= LINE_LOST)))
        {
            (*ppd_now)->state = ID_LOSE; //变为丢线模式
        }
    }

    else if ((*ppd_last)->state == ID_LEAVE_GAR) //出库模式
    {
        turn_speed_flag = 1;
        //TO_DO:
        //可能还要设一个count，让车出库前先走一小段直线距离，得看到图像才能确定需不需要
        error[4] = error[3] =
            error[2] = error[1] =
                error[0] = 0.0;

        (*ppd_now)->leave_garage_straight_count++;

        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);
        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;
        if ((*ppd_now)->leave_garage_straight_count >= LEAVE_GARAGE_STRAIGHT_COUNT)
        {
            error[4] = error[3] =
                error[2] = error[1] =
                    error[0] = LEAVE_GARAGE_ERROR;

            (*ppd_now)->l_g_count++;

            if ((*ppd_now)->l_g_count >= LEAVE_GARAGE_TURN_COUNT)
            {
                if (pl1->exist && pr1->exist && (lenl1 >= LINE_GET) && (lenr1 >= LINE_GET))
                {
                    error[4] = pl1->d - pr1->d;
                    (*ppd_now)->state = ID_STRAI; //变为直线模式
                }
                else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
                {
                    error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                    (*ppd_now)->state = ID_ONEL; //变为左单线模式
                }
                else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
                {
                    error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                    (*ppd_now)->state = ID_ONER; //变为右单线模式
                }
                else
                    (*ppd_now)->state = ID_LEAVE_GAR;
            }
            else
                (*ppd_now)->state = ID_LEAVE_GAR; //保持模式不变
        }
        else
            (*ppd_now)->state = ID_LEAVE_GAR; //保持模式不变
    }
    else if ((*ppd_last)->state == ID_FIRST_ZEBRA) //第一次入库斑马线
    {
        turn_speed_flag = 0;
        error[4] = error[3] =
            error[2] = error[1] =
                error[0] = 0.0;

        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);
        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;
        (*ppd_now)->first_zebra_count++;
        if ((*ppd_now)->first_zebra_count >= FIRST_ZEBRA_COUNT)
        {
            if (pl1->exist && pr1->exist && (lenl1 >= LINE_GET) && (lenr1 >= LINE_GET))
            {
                error[4] = pl1->d - pr1->d;
                (*ppd_now)->state = ID_STRAI; //变为直线模式
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //变为左单线模式
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //变为右单线模式
            }
            else
                (*ppd_now)->state = ID_FIRST_ZEBRA; //保持模式不变
        }
        else
            (*ppd_now)->state = ID_FIRST_ZEBRA; //保持模式不变
    }

    else if ((*ppd_last)->state == ID_ENTER_GAR) // 入库模式
    {
        turn_speed_flag = 1;
        //TO_DO:
        //可能还要设一个count，让车入库前先走一小段直线距离，得看到图像才能确定需不需要
        error[4] = error[3] =
            error[2] = error[1] =
                error[0] = ENTER_GARAGE_ERROR;

        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);
        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;

        (*ppd_now)->e_g_count++;

        if ((*ppd_now)->e_g_count >= ENTER_GARAGE_COUNT)
        {
            //入库后先保持直行模式，以便调整姿态，避免出界
            /*if (pl1->exist && pr1->exist && (lenl1 >= LINE_GET) && (lenr1 >= LINE_GET))
            {
                error[4] = pl1->d - pr1->d;
                (*ppd_now)->state = ID_STRAI; //变为直线模式
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //变为左单线模式
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //变为右单线模式
            }
            else
                (*ppd_now)->state = ID_ENTER_GAR;*/
            (*ppd_now)->state = ID_STOP;
        }
        else
            (*ppd_now)->state = ID_ENTER_GAR; //保持模式不变
    }

    else if ((*ppd_last)->state == ID_STOP) // 停车模式
    {
        pwm_stop_flag = 1;
        pwm_duty(MOTOR_PINA, 0);
        (*ppd_now)->state = ID_STOP;
        //停车且保持
    }

    else if ((*ppd_last)->state == ID_RIGHT_ENTER_ROUND) //右入环模式
    {

        error[4] = error[3] =
            error[2] = error[1] =
                error[0] = 0.0;

        (*ppd_now)->enter_straight_count++;

        if ((*ppd_now)->enter_straight_count >= ENTER_ROUND_STRAIGHT_COUNT)
        {
            error[4] = error[3] =
                error[2] = error[1] =
                    error[0] = RIGHT_ENTER_ROUND_ERROR;

            (*ppd_now)->enter_turn_count++;
            if ((*ppd_now)->enter_turn_count >= ENTER_ROUND_TURN_COUNT)
            {
                (*ppd_now)->state = ID_IN_ROUND; //变为环中模式
            }
            else
                (*ppd_now)->state = ID_RIGHT_ENTER_ROUND; //保持状态不变
        }
        else
            (*ppd_now)->state = ID_RIGHT_ENTER_ROUND; //保持状态不变
    }

    else if ((*ppd_last)->state == ID_LEFT_ENTER_ROUND) //左入环模式
    {
        error[4] = error[3] =
            error[2] = error[1] =
                error[0] = 0.0;

        (*ppd_now)->enter_straight_count++;

        if ((*ppd_now)->enter_straight_count >= ENTER_ROUND_STRAIGHT_COUNT)
        {
            error[4] = error[3] =
                error[2] = error[1] =
                    error[0] = LEFT_ENTER_ROUND_ERROR;

            (*ppd_now)->enter_turn_count++;
            if ((*ppd_now)->enter_turn_count >= ENTER_ROUND_TURN_COUNT)
            {
                (*ppd_now)->state = ID_IN_ROUND; //变为环中模式
            }
            else
                (*ppd_now)->state = ID_LEFT_ENTER_ROUND;
        }
        else
            (*ppd_now)->state = ID_LEFT_ENTER_ROUND;
    }

    else if ((*ppd_last)->state == ID_IN_ROUND) //环中模式
    {
        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);
        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;

        if (pl1->exist && pr1->exist && (lenl1 >= LINE_GET) && (lenr1 >= LINE_GET))
        {
            error[4] = pl1->d - pr1->d;
            (*ppd_now)->state = ID_STRAI; //变为直线模式
        }
        else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
        {
            error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
            (*ppd_now)->state = ID_ONEL; //变为左单线模式
        }
        else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
        {
            error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
            (*ppd_now)->state = ID_ONER; //变为右单线模式
        }
        else
            (*ppd_now)->state = ID_IN_ROUND; //保持模式不变
    }

    else if ((*ppd_last)->state == ID_ONER) //右单线模式
    {
        turn_speed_flag = 1;
        (*ppd_now)->center = (*ppd_last)->center;
        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);
        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;

        fork_check(src, pl1, pr1, lenl1, lenr1);

        if (pr1->exist && (lenr1 > LINE_LOST))
        {
            if (pl1->exist && (lenl1 >= LINE_GET))
            {
                error[4] = pl1->d - pr1->d;
                (*ppd_now)->state = ID_STRAI; //变为直线模式
            }
            else
            {
                line_qfit(*ppd_now, pr1);
                f1 = (float32)(pointj) - (float32)(pointi) * (*ppd_now)->qf_a[1] - (*ppd_now)->qf_a[0];
                f2 = 1.0 + (*ppd_now)->qf_a[1] * (*ppd_now)->qf_a[1];
                pr1->d = fabs(f1) / sqrt(f2);
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                if (error[4] < ONEE_MIN)
                    error[4] = ONEE_MIN;
                (*ppd_now)->state = ID_ONER; //模式不变
            }
        }
        else
        {
            error[4] = 0.0;
            if ((*ppd_last)->r1.lost <= 2)
                (*ppd_now)->state = ID_CLIMB; //变为爬坡模式
            else
                (*ppd_now)->state = ID_LOSE; //变为丢线模式
        }
    }

    else if ((*ppd_last)->state == ID_ONEL) //左单线模式
    {
        turn_speed_flag = 1;
        (*ppd_now)->center = (*ppd_last)->center;
        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);
        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;

        fork_check(src, pl1, pr1, lenl1, lenr1);

        if (pl1->exist && (lenl1 > LINE_LOST))
        {
            if (pr1->exist && (lenr1 >= LINE_GET))
            {
                error[4] = pl1->d - pr1->d;
                (*ppd_now)->state = ID_STRAI; //变为直线模式
            }
            else
            {
                line_qfit(*ppd_now, pl1);
                f1 = (float32)(pointj) - (float32)(pointi) * (*ppd_now)->qf_a[1] - (*ppd_now)->qf_a[0];
                f2 = 1.0 + (*ppd_now)->qf_a[1] * (*ppd_now)->qf_a[1];
                pl1->d = fabs(f1) / sqrt(f2);
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                if (error[4] > ONEE_MAX)
                    error[4] = ONEE_MAX;
                (*ppd_now)->state = ID_ONEL; //模式不变
            }
        }
        else //当左线也消失时
        {
            error[4] = 0.0;
            int white_of_lost_row = 0, i;
            for (i = 5; i < 90; i++)
            {
                if (f(src, (*ppd_last)->l1.lost, i, IMG_W) == 255)
                    white_of_lost_row++;
            }
            if ((*ppd_last)->l1.lost <= 2 && white_of_lost_row < 10)
            //边线的lost最小为1，则仅当lost为1或2时判断为爬坡
            {
                (*ppd_now)->c_count = 0;
                (*ppd_now)->state = ID_CLIMB; //变为爬坡模式
            }
            /*
            区分爬坡和丢线的思路：
            看左单线模式下左线是怎么丢的，
            如果是像十字路或三岔那样线“突然截断”
            那么丢线的行数应该在屏幕中间处；
            但如果是爬坡丢线，
            则是因为视野受限而使左边线“逐渐丢掉”，
            则lost应该是个很小的值

            还有一个地方和爬坡的“逐渐丢线”很像：刚进十字路口的时候
            所以要再加一个判断，就是丢线那一行不能是白、黑、白三段
            */
            else
                (*ppd_now)->state = ID_LOSE; //变为丢线模式
        }
    }

    else if ((*ppd_last)->state == ID_LOSE) //丢线模式
    {
        col_l = 1;
        col_r = IMG_W - 2;

        turn_speed_flag = 1;
        /* int16 col_l, col_r;*/

        //丢线模式的error设为0，则短暂直行
        error[4] = error[3] =
            error[2] = error[1] =
                error[0] = 0.0;

        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);

        //直线模式和丢线模式都要判断有无出库
        /*  leave_zebra(src, pointi, pointj);*/

        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;

        //这三种模式选择中包含了十字路口处理：丢线模式一直直行，直至进入左单线或者右单线
        /*if (leave_jump_count >= LEAVE_JUMP_COUNT)
        { //竖直方向的跳变点足够多时进入出库模式
            leave_jump_count = 0;
            (*ppd_now)->l_g_count = 0;
            (*ppd_now)->state = ID_LEAVE_GAR;
        }*/

        fork_dianci_check(induc_value);

        if (pl1->exist && pr1->exist && (lenl1 >= LINE_GET) && (lenr1 >= LINE_GET))
        {
            error[4] = pl1->d - pr1->d;
            (*ppd_now)->state = ID_STRAI; //变为直线模式
        }
        else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
        {
            error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
            (*ppd_now)->state = ID_ONEL; //变为左单线模式
        }
        else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
        {
            error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
            (*ppd_now)->state = ID_ONER; //变为右单线模式
        }

        //只有此前在直线或单线模式中检测到了两边的凸出，才能在接下来的丢线模式中进一步判断三岔
        else /*if (fork_check_flag_1 == 1)*/
        {
            /*if (!pl1->exist)
                col_l = 1;
            else
            {
                lenl1 = pl1->begin - pl1->lost + 1;
                col_l = pl1->pos[lenl1 - 1] + 1;
            }
            if (!pr1->exist)
                col_r = IMG_W - 2;
            else
            {
                lenr1 = pr1->begin - pr1->lost + 1;
                col_r = pr1->pos[lenr1 - 1] - 1;
            }

            //col_l和col_r代表两边白线递增的终止位置
            //把两终止位置作为起点和终点，将连线8等分
            (*ppd_now)->tri_j[0] = (float32)(col_l)*1.0 / 9.0 + (float32)(col_r)*8.0 / 9.0;
            (*ppd_now)->tri_j[1] = (float32)(col_l)*2.0 / 9.0 + (float32)(col_r)*7.0 / 9.0;
            (*ppd_now)->tri_j[2] = (float32)(col_l)*3.0 / 9.0 + (float32)(col_r)*6.0 / 9.0;
            (*ppd_now)->tri_j[3] = (float32)(col_l)*4.0 / 9.0 + (float32)(col_r)*5.0 / 9.0;
            (*ppd_now)->tri_j[4] = (float32)(col_l)*5.0 / 9.0 + (float32)(col_r)*4.0 / 9.0;
            (*ppd_now)->tri_j[5] = (float32)(col_l)*6.0 / 9.0 + (float32)(col_r)*3.0 / 9.0;
            (*ppd_now)->tri_j[6] = (float32)(col_l)*7.0 / 9.0 + (float32)(col_r)*2.0 / 9.0;
            (*ppd_now)->tri_j[7] = (float32)(col_l)*8.0 / 9.0 + (float32)(col_r)*1.0 / 9.0;
            (*ppd_now)->tri_sum = 0.0;

            int max = 0;
            max_col = 0;
            //从每个等分点向上找到第一个白点（即三岔路的凸起白线）
            for (i = 0; i < 8; i++)
            {
                j = (int16)(*ppd_now)->tri_j[i];
                i1 = IMG_H / 2;
                while (i1 > 0 && f(src, i1, j, IMG_W) == 0)
                    i1--;
                (*ppd_now)->tri_i[i] = (float32)(i1);
                (*ppd_now)->tri_sum += (*ppd_now)->tri_i[i] * (*ppd_now)->tri_w[i];

                if (i1 > max)
                {
                    max = i1;
                    max_col = (*ppd_now)->tri_j[i];
                }
                //找出三角尖的列数

                //tri_w[]是固定权值，两边小，中间大
            }

            //顺便用这个数组做出环处理
            for (i = 1; i < 9; i++)
            {
                if ((*ppd_now)->tri_i[i] < (*ppd_now)->tri_i[i - 1])
                { //右出环检线
                    right_leave_round_point++;
                }

                if ((*ppd_now)->tri_i[i] > (*ppd_now)->tri_i[i - 1])
                { //左出环检线
                    left_leave_round_point++;
                }
            }*/

            //大于某个阈值时说明中间突起很明显，确实是个三角形状
           /* if ((*ppd_now)->tri_sum > TRI_THSD /*&& max_col > 30 && max_col < 65)*/
            if(dianci_enter_fork_flag==1)
           {
               fork_check_flag = 1;
               if ((*ppd_now)->f_dir == F_DIR_R)
                   error[4] = error[3] =
                       error[2] = error[1] =
                           error[0] = F_R_ERROR;
               else
                   error[4] = error[3] =
                       error[2] = error[1] =
                           error[0] = F_L_ERROR;
               (*ppd_now)->f_count = 0;
               (*ppd_now)->state = ID_FORK; //变为三岔路口模式
               dianci_enter_fork_flag = 0;
               fork_check_flag_1 = 0;
           }

           else if (dianci_right_leave_fork_flag == 1)
           {
               (*ppd_now)->f_count = 0;
               (*ppd_now)->state = ID_R_LEAVE_FORK;//变为右出三岔模式
               dianci_enter_fork_flag = 0;
           }

           else if (dianci_left_leave_fork_flag == 1)
           {
               (*ppd_now)->f_count = 0;
               (*ppd_now)->state = ID_L_LEAVE_FORK;//变为左出三岔模式
               dianci_enter_fork_flag = 0;
           }

           else if (/*right_leave_round_point >= LEAVE_ROUND_CHECK_LINE*/ right_leave_round_flag == 1)
           {
               right_leave_round_point = 0;
               right_leave_round_flag = 0;
               (*ppd_now)->state = ID_RIGHT_LEAVE_ROUND; //变为右出环模式
           }

           else if (/*left_leave_round_point >= LEAVE_ROUND_CHECK_LINE*/ left_leave_round_flag == 1)
           {
               left_leave_round_point = 0;
               left_leave_round_flag = 0;
               (*ppd_now)->state = ID_LEFT_LEAVE_ROUND; //变为左出环模式
           }

           else
           {
               right_leave_round_point = 0;
               left_leave_round_point = 0;
               (*ppd_now)->state = ID_LOSE; //维持模式不变
               fork_check_flag_1 = 1;
           }
        }
    }

    else if ((*ppd_last)->state == ID_FORK) //三岔路口模式
    {
        turn_speed_flag = 1;
        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);
        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;

        (*ppd_now)->f_count++; //f_count用于保持三岔路口状态一段时间

        if ((*ppd_now)->f_dir == F_DIR_R)
        {
            error[4] = error[3] =
                error[2] = error[1] =
                    error[0] = F_R_ERROR; //error为负时servoduty变小，右转
            if ((*ppd_now)->f_count >= F_COUNTS)
            {
                (*ppd_now)->f_time++;

                //周期循环：右右左左
                if ((*ppd_now)->f_time >= 1)
                {
                    (*ppd_now)->f_time = 0;
                    (*ppd_now)->f_dir = F_DIR_L;
                }

                error[4] = error[3] = error[2] =
                    error[1] = error[0] = 0.0;

                //跑完三岔路维持周期后再次进入丢线状态，以便重新判断正常路段
                (*ppd_now)->state = ID_LOSE;
                fork_check_flag = 0;
            }
            else
                (*ppd_now)->state = ID_FORK; //维持模式不变
        }
        else
        {
            error[4] = error[3] =
                error[2] = error[1] =
                    error[0] = F_L_ERROR;
            if ((*ppd_now)->f_count >= F_COUNTS)
            {
                (*ppd_now)->f_time++;
                if ((*ppd_now)->f_time >= 1)
                {
                    (*ppd_now)->f_time = 0;
                    (*ppd_now)->f_dir = F_DIR_R;
                }
                error[4] = error[3] = error[2] =
                    error[1] = error[0] = 0.0;

                (*ppd_now)->state = ID_LOSE;
            }
            else
                (*ppd_now)->state = ID_FORK; //维持模式不变
        }
    }

    else if ((*ppd_last)->state == ID_R_LEAVE_FORK) //右出三岔
    {
        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);
        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;

        (*ppd_now)->f_count++;

        error[4] = error[3] =
            error[2] = error[1] =
                error[0] = F_R_ERROR;

        if ((*ppd_now)->f_count >= F_COUNTS)
        {
            if (pl1->exist && pr1->exist && (lenl1 >= LINE_GET) && (lenr1 >= LINE_GET))
            {
                error[4] = pl1->d - pr1->d;
                (*ppd_now)->state = ID_STRAI; //变为直线模式
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //变为左单线模式
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //变为右单线模式
            }
            else
                (*ppd_now)->state = ID_R_LEAVE_FORK;
        }
        else
            (*ppd_now)->state = ID_R_LEAVE_FORK;
    }

    else if ((*ppd_last)->state == ID_L_LEAVE_FORK) //左出三岔
    {
        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);
        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;

        (*ppd_now)->f_count++;

        error[4] = error[3] =
            error[2] = error[1] =
                error[0] = F_L_ERROR;

        if ((*ppd_now)->f_count >= F_COUNTS)
        {
            if (pl1->exist && pr1->exist && (lenl1 >= LINE_GET) && (lenr1 >= LINE_GET))
            {
                error[4] = pl1->d - pr1->d;
                (*ppd_now)->state = ID_STRAI; //变为直线模式
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //变为左单线模式
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //变为右单线模式
            }
            else
                (*ppd_now)->state = ID_L_LEAVE_FORK;
        }
        else
            (*ppd_now)->state = ID_L_LEAVE_FORK;
    }

    else if ((*ppd_last)->state == ID_RIGHT_LEAVE_ROUND) //右出环模式
    {
        error[4] = error[3] =
            error[2] = error[1] =
                error[0] = RIGHT_LEAVE_ROUND_ERROR;

        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);
        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;
        (*ppd_now)->leave_turn_count++;

        if ((*ppd_now)->leave_turn_count >= LEAVE_ROUND_COUNT)
        {
            if (pl1->exist && pr1->exist && (lenl1 >= LINE_GET) && (lenr1 >= LINE_GET))
            {
                error[4] = pl1->d - pr1->d;
                (*ppd_now)->state = ID_STRAI; //变为直线模式
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //变为左单线模式
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //变为右单线模式
            }
            else
                (*ppd_now)->state = ID_RIGHT_LEAVE_ROUND; //保持模式不变
        }
        else
            (*ppd_now)->state = ID_RIGHT_LEAVE_ROUND; //保持模式不变
    }

    else if ((*ppd_last)->state == ID_LEFT_LEAVE_ROUND) //左出环模式
    {
        error[4] = error[3] =
            error[2] = error[1] =
                error[0] = LEFT_LEAVE_ROUND_ERROR;

        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);
        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;
        (*ppd_now)->leave_turn_count++;

        if ((*ppd_now)->leave_turn_count >= LEAVE_ROUND_COUNT)
        {
            if (pl1->exist && pr1->exist && (lenl1 >= LINE_GET) && (lenr1 >= LINE_GET))
            {
                error[4] = pl1->d - pr1->d;
                (*ppd_now)->state = ID_STRAI; //变为直线模式
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //变为左单线模式
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //变为右单线模式
            }
            else
                (*ppd_now)->state = ID_LEFT_LEAVE_ROUND; //保持模式不变
        }
        else
            (*ppd_now)->state = ID_LEFT_LEAVE_ROUND; //保持模式不变
    }

    //注意：爬坡时也是会丢线的
    else if ((*ppd_last)->state == ID_CLIMB) //爬坡模式
    {
        turn_speed_flag = 0;
        error[4] = error[3] =
            error[2] = error[1] =
                error[0] = 0.0;

        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);
        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;
        (*ppd_now)->c_count++;
        if ((*ppd_now)->c_count >= C_COUNTS)
        {
            if (pl1->exist && pr1->exist && (lenl1 >= LINE_GET) && (lenr1 >= LINE_GET))
            {
                error[4] = pl1->d - pr1->d;
                (*ppd_now)->state = ID_STRAI; //变为直线模式
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //变为左单线模式
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //变为右单线模式
            }
            else
                (*ppd_now)->state = ID_CLIMB; //保持模式不变
        }
        else
            (*ppd_now)->state = ID_CLIMB; //保持模式不变
    }
}

void id_getleft(uint8 *src, ImgData *pd)
{
    int16 row, col, sub; //行，列，下标
    int16 col_info[4];
    uint8 draw_state;
    row = IMG_H - 2;
    col = IMG_W / 2;
    sub = 0;

    //找到左边线最下面那行的最右边的那个白点
    while ((f(src, row, col, IMG_W) == 0) && (col > 0))
        col--;

    col_info[0] = col_info[1] =
        col_info[2] = col_info[3] = col;
    draw_state = 0;

    while (row >= 1)
    {
        row--;
        col += 3;

        //行数从近到远，找到左边线每一行的最右边的那个白点
        while (f(src, row, col, IMG_W) && (col < (IMG_W - 1)))
            col++;
        while ((f(src, row, col, IMG_W) == 0) && (col > 0))
            col--;

        col_info[0] = col_info[1];
        col_info[1] = col_info[2];
        col_info[2] = col_info[3];
        col_info[3] = col;

        if (draw_state == 0)
        {
            if (col_info[3] > col_info[1]) //白线正常递增
            {
                draw_state = 1;
                pd->l1.exist = 1;
                pd->l1.begin = row;
                sub = 0;
            }
        }
        else
        {
            if (col_info[3] < col_info[0])
            {
                pd->l1.lost = row + 1;
                break;
            }
        }
        if (sub >= LINE_LEN)
        {
            if (draw_state == 1)
                pd->l1.lost = row + 1;
            break;
        }
        if (draw_state == 1)
        {
            pd->l1.pos[sub] = col;
            sub++;
        }
    }
    if (row < 1)
    {
        if (draw_state == 1)
            pd->l1.lost = row + 1;
    }
}

void id_getright(uint8 *src, ImgData *pd)
{
    int16 row, col, sub; //行，列，下标
    int16 col_info[4];
    uint8 draw_state;
    row = IMG_H - 2;
    col = IMG_W / 2;
    sub = 0;
    while ((f(src, row, col, IMG_W) == 0) && (col > 0))
        col++;
    col_info[0] = col_info[1] =
        col_info[2] = col_info[3] = col;
    draw_state = 0;
    while (row >= 3)
    {
        row--;
        col -= 3;
        while (f(src, row, col, IMG_W) && (col > 0))
            col--;
        while ((f(src, row, col, IMG_W) == 0) && (col < (IMG_W - 1)))
            col++;
        col_info[0] = col_info[1];
        col_info[1] = col_info[2];
        col_info[2] = col_info[3];
        col_info[3] = col;
        if (draw_state == 0)
        {
            if (col_info[3] < col_info[1])
            {
                draw_state = 1;
                pd->r1.exist = 1;
                pd->r1.begin = row;
                sub = 0;
            }
        }
        else
        {
            if (col_info[3] > col_info[0])
            {
                pd->r1.lost = row + 1;
                break;
            }
        }
        if (sub >= LINE_LEN)
        {
            if (draw_state == 1)
                pd->r1.lost = row + 1;
            break;
        }
        if (draw_state == 1)
        {
            pd->r1.pos[sub] = col;
            sub++;
        }
    }
    if (row < 3)
    {
        if (draw_state == 1)
            pd->r1.lost = row + 1;
    }
}

void id2img(ImgData *pd, uint8 *sob, uint16 width, uint16 height)
{
    int16 i, j;
    if (pd->l1.exist)
    {
        for (i = pd->l1.begin, j = 0; i >= pd->l1.lost; i--, j++)
        {
            if ((pd->l1.pos[j] > 0) && (pd->l1.pos[j] < width - 1))
                f(sob, i, pd->l1.pos[j], width) = 1;
        }
        i = IMG_W / 2 - 5;
        for (j = 1; j < i; j++)
        {
            f(sob, pd->l1.begin, j, width) =
                f(sob, pd->l1.lost, j, width) = 1;
        }
    }
    if (pd->r1.exist)
    {
        for (i = pd->r1.begin, j = 0; i >= pd->r1.lost; i--, j++)
        {
            if ((pd->r1.pos[j] > 0) && (pd->r1.pos[j] < width - 1))
                f(sob, i, pd->r1.pos[j], width) = 2;
        }
        i = IMG_W / 2 + 5;
        for (j = IMG_W - 2; j > i; j--)
        {
            f(sob, pd->r1.begin, j, width) =
                f(sob, pd->r1.lost, j, width) = 2;
        }
    }
}

void line_getkd(Line *pl, uint16 i, uint16 j)
{
    uint16 len;
    float32 f1, f2;
    if (pl->exist)
    {
        len = pl->begin - pl->lost + 1;
        if (len > 3)
        {
            pl->k = (float32)(pl->pos[len - 2] - pl->pos[1]) / (float32)(len - 3);
            pl->k *= -1.0;
            f1 = (float32)(pl->begin - 1);
            f2 = (float32)(pl->pos[1]);
            f2 -= (f1 * pl->k); //f2=b
            pl->d = fabs((float32)(j) - (float32)(i)*pl->k - f2) / sqrt(1 + pl->k * pl->k);
        }
    }
}

void line_qfit(ImgData *pd, Line *pl)
{
    int16 i, in;
    float32 delta, dt[2];
    pd->qf_b[0] = pd->qf_b[1] =
        pd->qf_b[2] = 0.0;
    pd->qf_c[0] = pd->qf_c[1] = 0.0;
    for (in = pl->begin, i = 0; in >= pl->lost; in--, i++)
    {
        pd->qf_b[0] += 1.0;
        pd->qf_b[1] += in;
        pd->qf_b[2] += in * in;
        pd->qf_c[0] += pl->pos[i];
        pd->qf_c[1] += pl->pos[i] * in;
    }
    delta = pd->qf_b[0] * pd->qf_b[2] - pd->qf_b[1] * pd->qf_b[1];
    dt[0] = pd->qf_c[0] * pd->qf_b[2] - pd->qf_c[1] * pd->qf_b[1];
    dt[1] = pd->qf_c[1] * pd->qf_b[0] - pd->qf_c[0] * pd->qf_b[1];
    pd->qf_a[0] = dt[0] / delta;
    pd->qf_a[1] = dt[1] / delta;
}

void leave_zebra(uint8 *src, uint16 i, uint16 j)
{
    int16 row;
    //TO_DO：斑马线判断准则可能需完善，如跳变之后/之前要有两个点相同
    for (row = i; row >= 5; row--)
    { //从底边中线往上找跳变点
        if (ABS(f(src, row, j, IMG_W) - f(src, row - 1, j, IMG_W)) == 255 /*&& (f(src, row - 1, j, IMG_W) == f(src, row - 2, j, IMG_W))*/)
        {
            leave_jump_count++;
        }
    }
    if (leave_jump_count < LEAVE_JUMP_COUNT)
    {
        leave_jump_count = 0;
    }
}

void enter_zebra(uint8 *src, uint16 i, uint16 j, ImgData **ppd_last, ImgData **ppd_now)
{ //中间行往右找跳变点
    int16 row, col;
    row = i / 2;

    for (col = 20; col <= 67; col++)
    {
        //白跳变为黑，且黑点连续，且跳变个数足够多
        if ((f(src, row, col, IMG_W) == 255 &&
             (f(src, row - 1, col, IMG_W) == 255 || f(src, row + 1, col, IMG_W) == 255) &&
             f(src, row, col + 1, IMG_W)) == 0 &&
            f(src, row, col + 2, IMG_W) == 0 &&
            f(src, row, col + 3, IMG_W) == 0)
        {
            enter_jump_count++;
        }
    }

    //据观察，在20-70列时符合上述标准的跳变点一般有4或5个
    if (enter_jump_count >= 25 && enter_jump_count <= 30)
    {
        /*systick_delay_ms(STM1,300);*/
        //跳变点大于阈值后直接return，以免enter_gar_time多次增加
        enter_gar_time++;
        if (enter_gar_time == 1)
        {
            (*ppd_now)->l_g_count = 0;
            (*ppd_now)->state = ID_FIRST_ZEBRA;
        }
        enter_jump_count = 0;
        return;
    }
    enter_jump_count = 0;
}

void check_stop(uint8 *src, uint16 i, uint16 j)
{
    //TO_DO：停车判断准则可能需完善
    int row = i, k, count;

    //找到底边中点往上走的第一个白点
    while (f(src, row, j, IMG_W) == 0 && row > 5)
        row--;

    if (f(src, row, j, IMG_W) == 255 /* && f(src, row - 1, j, IMG_W) == 255*/)
    {
        //共找30个点
        for (k = 18; k < IMG_W - 18; k = k + 2)
        {
            if (f(src, row, k, IMG_W) == 255)
                count++;
        }
    }
    if (count > STOP_POINT_THRS)
    {
        stop_flag = 1;
    }
}

void fork_check(uint8 *src, Line *pl, Line *pr, uint16 lenl1, uint16 lenr1)
{
    int i, j;
    int col[4];

    if (!pl->exist)
        col_l = 1;
    else
    {
        lenl1 = pl->begin - pl->lost + 1;
        col_l = pl->pos[lenl1 - 1] + 1;
    }
    if (!pr->exist)
        col_r = IMG_W - 2;
    else
    {
        lenr1 = pr->begin - pr->lost + 1;
        col_r = pr->pos[lenr1 - 1] - 1;
    }

    col[0] = col_l + 5;
    col[1] = col_l + 6;
    col[2] = col_l + 7;
    // col[3] = col_l - 3+5;
    for (i = pl->lost - 1, j = 0; i > pl->lost - 4; i--, j++)
    {
        while (f(src, i, col[j], IMG_W) == 0 && (col_l - col[j] < 10 + j) && col[j] > 1)
            col[j]--;
        if (col_l - col[j] >= 10 + j || col[j] == 1 || col_l - j - col[j] <= 0)
        {
            l_angle_flag++; //不满足的点每来一个则加一
        }
    }

    col[0] = col_r - 5;
    col[1] = col_r - 6;
    col[2] = col_r - 7;
    //  col[3] = col_r + 3;
    for (i = pr->lost - 1, j = 0; i > pr->lost - 4; i--, j++)
    {
        while (f(src, i, col[j], IMG_W) == 0 && (col[j] - col_r < 10 + j) && col[j] < IMG_W - 1)
            col[j]++;
        if (col[j] - col_r >= 10 + j || col[j] == IMG_W - 1 || col[j] - col_r - j <= 0)
        {
            r_angle_flag++;
        }
    }

    if ((pl->lost > 3 &&
         col_l > 13 &&
         l_angle_flag <= 2) ||
        (pr->lost > 3 &&
         col_r < 79 && //检测的4个点中，不满足的点最多允许有一个
         r_angle_flag <= 2))
    {
        /*systick_delay_ms(STM1, 333);*/
        fork_check_flag_1 = 1;
        l_angle_flag = 0;
        r_angle_flag = 0;
    }

    /*  if ((lenl1 >3 &&
         col_l > 20 &&
         l_angle_flag < 1) ||
        (lenr1 > 3 &&
         col_r < 70 && //检测的4个点中，不满足的点最多允许有一个
         r_angle_flag < 1))
    {
        fork_check_flag_1 = 1;
    }*/
    /* systick_delay_ms(STM1, 333);*/
    l_angle_flag = 0;
    r_angle_flag = 0;
}

void round_check(int16 *value)
{ //TO_DO:需要把判定标准改成两个横电感都大于4000吗？
    if (value[0] < 500 && (value[1] > 4000 && value[3] > 4000) && value[4] > 1000 && value[2] < 800)
    {
        right_enter_round_flag = 1;
        right_leave_round_flag = 1;
    }
    if (value[4] < 500 && (value[1] > 4000 && value[3] > 4000) && value[0] > 1000 && value[2] < 800)
    {
        left_enter_round_flag = 1;
        left_leave_round_flag = 1;
    }
}

void fork_dianci_check(int16 *value)
{
    if (value[0] < 1000 &&
        (value[1] > 400 && value[1] < 800) &&
        value[2] < 500 &&
        (value[3] > 400 && value[3] < 800) &&
        value[4] < 1000)
    {
        dianci_enter_fork_flag = 1;
    }

    if ((value[0] > 1500 && value[0] < 2400) &&
        value[1] < 400 &&
        (value[2] > 1900 && value[2] < 2900) &&
        (value[4] > 2500 && value[4] < 3500))
    {
        dianci_right_leave_fork_flag = 1;
    }

    if ((value[4] > 1500 && value[4] < 2400) &&
        value[3] < 400 &&
        (value[2] > 1900 && value[2] < 2900) &&
        (value[0] > 2500 && value[0] < 3500))
    {
        dianci_left_leave_fork_flag = 1;
    }
}
