/*
 * imgp.c
 *
 *  Created on: 2021��5��20��
 *      Author: �콭��
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

float32 sobel_thsf; //��ȷ����ֵ
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
    uint16 w1, h1; //�߽�ֵ
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

    //����ָ�룬����һ��ָ��ĵ�ַ����ʡʱ��ռ�
    p0 = *ppd_last;
    *ppd_last = *ppd_now;
    *ppd_now = p0;

    //now�����ݸ���last���ٰ�now����ʼ�����������ڼ�����count��time֮���ȫ������last
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
    //���ĸ�error��Ϊ�˺���ʹ��Iϵ������PD������ֻ�õ�error[3]��error[4]

    pl1 = &((*ppd_now)->l1);
    pr1 = &((*ppd_now)->r1);

    if ((*ppd_last)->state == ID_STRAI) //ֱ��ģʽ
    {
        turn_speed_flag = 0;
        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);

        //�õ�ֱ�ߵ�б�ʺ͵��ױ��е�ľ���
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);

        lenl1 = pl1->begin - pl1->lost + 1; //ָ��ֱ�����ϵĳ���
        lenr1 = pr1->begin - pr1->lost + 1;
        //�����ߺ�ͣ��check
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
        { //���Ҷ�δ�����Ҳ�����ⲻ���뻷
            error[4] = pl1->d - pr1->d;
            (*ppd_now)->state = ID_STRAI; //ģʽ����
        }

        /*else if (leave_jump_count >= LEAVE_JUMP_COUNT)
        { //��ֱ�����������㹻��ʱ�������ģʽ
            leave_jump_count = 0;
            (*ppd_now)->l_g_count = 0;
            (*ppd_now)->state = ID_LEAVE_GAR;
        }*/

        else if (enter_gar_time >= 2)
        { //��������߳�������ʱ�������ģʽ
            enter_gar_time = 0;
            (*ppd_now)->e_g_count = 0;
            (*ppd_now)->state = ID_ENTER_GAR;
        }

        /* else if (stop_flag == 1)
        { //��Ϊͣ��ģʽ
            stop_flag = 0;
            (*ppd_now)->state = ID_STOP;
        }*/

        else if (right_enter_round_flag == 1)
        { //��Ϊ���뻷ģʽ
            right_enter_round_flag = 0;
            (*ppd_now)->enter_straight_count = 0;
            (*ppd_now)->enter_turn_count = 0;
            (*ppd_now)->state = ID_RIGHT_ENTER_ROUND;
        }

        else if (left_enter_round_flag == 1)
        { //��Ϊ���뻷ģʽ
            left_enter_round_flag = 0;
            (*ppd_now)->enter_straight_count = 0;
            (*ppd_now)->enter_turn_count = 0;
            (*ppd_now)->state = ID_LEFT_ENTER_ROUND;
        }

        else if ((!pl1->exist) || (lenl1 <= LINE_LOST))
        { //����else-if��˳���ܽ������if�жϵĻ���ǰ�漸������Ԫ�ص������ز�����

            //error�ľ����㷨��
            (*ppd_now)->center = (0.5 * error[3] + (*ppd_last)->r1.d) * 0.95;
            if ((*ppd_now)->center > CEN_MAX)
                (*ppd_now)->center = CEN_MAX;
            else if ((*ppd_now)->center < CEN_MIN)
                (*ppd_now)->center = CEN_MIN;
            error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
            if (error[4] < ONEE_MIN)
                error[4] = ONEE_MIN;
            (*ppd_now)->state = ID_ONER; //��Ϊ�ҵ���ģʽ
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
            (*ppd_now)->state = ID_ONEL; //��Ϊ����ģʽ
        }

        else if (!(pl1->exist || (lenl1 <= LINE_LOST)) && !(pr1->exist || (lenr1 <= LINE_LOST)))
        {
            (*ppd_now)->state = ID_LOSE; //��Ϊ����ģʽ
        }
    }

    else if ((*ppd_last)->state == ID_LEAVE_GAR) //����ģʽ
    {
        turn_speed_flag = 1;
        //TO_DO:
        //���ܻ�Ҫ��һ��count���ó�����ǰ����һС��ֱ�߾��룬�ÿ���ͼ�����ȷ���費��Ҫ
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
                    (*ppd_now)->state = ID_STRAI; //��Ϊֱ��ģʽ
                }
                else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
                {
                    error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                    (*ppd_now)->state = ID_ONEL; //��Ϊ����ģʽ
                }
                else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
                {
                    error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                    (*ppd_now)->state = ID_ONER; //��Ϊ�ҵ���ģʽ
                }
                else
                    (*ppd_now)->state = ID_LEAVE_GAR;
            }
            else
                (*ppd_now)->state = ID_LEAVE_GAR; //����ģʽ����
        }
        else
            (*ppd_now)->state = ID_LEAVE_GAR; //����ģʽ����
    }
    else if ((*ppd_last)->state == ID_FIRST_ZEBRA) //��һ����������
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
                (*ppd_now)->state = ID_STRAI; //��Ϊֱ��ģʽ
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //��Ϊ����ģʽ
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //��Ϊ�ҵ���ģʽ
            }
            else
                (*ppd_now)->state = ID_FIRST_ZEBRA; //����ģʽ����
        }
        else
            (*ppd_now)->state = ID_FIRST_ZEBRA; //����ģʽ����
    }

    else if ((*ppd_last)->state == ID_ENTER_GAR) // ���ģʽ
    {
        turn_speed_flag = 1;
        //TO_DO:
        //���ܻ�Ҫ��һ��count���ó����ǰ����һС��ֱ�߾��룬�ÿ���ͼ�����ȷ���費��Ҫ
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
            //�����ȱ���ֱ��ģʽ���Ա������̬���������
            /*if (pl1->exist && pr1->exist && (lenl1 >= LINE_GET) && (lenr1 >= LINE_GET))
            {
                error[4] = pl1->d - pr1->d;
                (*ppd_now)->state = ID_STRAI; //��Ϊֱ��ģʽ
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //��Ϊ����ģʽ
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //��Ϊ�ҵ���ģʽ
            }
            else
                (*ppd_now)->state = ID_ENTER_GAR;*/
            (*ppd_now)->state = ID_STOP;
        }
        else
            (*ppd_now)->state = ID_ENTER_GAR; //����ģʽ����
    }

    else if ((*ppd_last)->state == ID_STOP) // ͣ��ģʽ
    {
        pwm_stop_flag = 1;
        pwm_duty(MOTOR_PINA, 0);
        (*ppd_now)->state = ID_STOP;
        //ͣ���ұ���
    }

    else if ((*ppd_last)->state == ID_RIGHT_ENTER_ROUND) //���뻷ģʽ
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
                (*ppd_now)->state = ID_IN_ROUND; //��Ϊ����ģʽ
            }
            else
                (*ppd_now)->state = ID_RIGHT_ENTER_ROUND; //����״̬����
        }
        else
            (*ppd_now)->state = ID_RIGHT_ENTER_ROUND; //����״̬����
    }

    else if ((*ppd_last)->state == ID_LEFT_ENTER_ROUND) //���뻷ģʽ
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
                (*ppd_now)->state = ID_IN_ROUND; //��Ϊ����ģʽ
            }
            else
                (*ppd_now)->state = ID_LEFT_ENTER_ROUND;
        }
        else
            (*ppd_now)->state = ID_LEFT_ENTER_ROUND;
    }

    else if ((*ppd_last)->state == ID_IN_ROUND) //����ģʽ
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
            (*ppd_now)->state = ID_STRAI; //��Ϊֱ��ģʽ
        }
        else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
        {
            error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
            (*ppd_now)->state = ID_ONEL; //��Ϊ����ģʽ
        }
        else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
        {
            error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
            (*ppd_now)->state = ID_ONER; //��Ϊ�ҵ���ģʽ
        }
        else
            (*ppd_now)->state = ID_IN_ROUND; //����ģʽ����
    }

    else if ((*ppd_last)->state == ID_ONER) //�ҵ���ģʽ
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
                (*ppd_now)->state = ID_STRAI; //��Ϊֱ��ģʽ
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
                (*ppd_now)->state = ID_ONER; //ģʽ����
            }
        }
        else
        {
            error[4] = 0.0;
            if ((*ppd_last)->r1.lost <= 2)
                (*ppd_now)->state = ID_CLIMB; //��Ϊ����ģʽ
            else
                (*ppd_now)->state = ID_LOSE; //��Ϊ����ģʽ
        }
    }

    else if ((*ppd_last)->state == ID_ONEL) //����ģʽ
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
                (*ppd_now)->state = ID_STRAI; //��Ϊֱ��ģʽ
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
                (*ppd_now)->state = ID_ONEL; //ģʽ����
            }
        }
        else //������Ҳ��ʧʱ
        {
            error[4] = 0.0;
            int white_of_lost_row = 0, i;
            for (i = 5; i < 90; i++)
            {
                if (f(src, (*ppd_last)->l1.lost, i, IMG_W) == 255)
                    white_of_lost_row++;
            }
            if ((*ppd_last)->l1.lost <= 2 && white_of_lost_row < 10)
            //���ߵ�lost��СΪ1�������lostΪ1��2ʱ�ж�Ϊ����
            {
                (*ppd_now)->c_count = 0;
                (*ppd_now)->state = ID_CLIMB; //��Ϊ����ģʽ
            }
            /*
            �������ºͶ��ߵ�˼·��
            ������ģʽ����������ô���ģ�
            �������ʮ��·�����������ߡ�ͻȻ�ضϡ�
            ��ô���ߵ�����Ӧ������Ļ�м䴦��
            ����������¶��ߣ�
            ������Ϊ��Ұ���޶�ʹ����ߡ��𽥶�������
            ��lostӦ���Ǹ���С��ֵ

            ����һ���ط������µġ��𽥶��ߡ����񣺸ս�ʮ��·�ڵ�ʱ��
            ����Ҫ�ټ�һ���жϣ����Ƕ�����һ�в����ǰס��ڡ�������
            */
            else
                (*ppd_now)->state = ID_LOSE; //��Ϊ����ģʽ
        }
    }

    else if ((*ppd_last)->state == ID_LOSE) //����ģʽ
    {
        col_l = 1;
        col_r = IMG_W - 2;

        turn_speed_flag = 1;
        /* int16 col_l, col_r;*/

        //����ģʽ��error��Ϊ0�������ֱ��
        error[4] = error[3] =
            error[2] = error[1] =
                error[0] = 0.0;

        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);

        //ֱ��ģʽ�Ͷ���ģʽ��Ҫ�ж����޳���
        /*  leave_zebra(src, pointi, pointj);*/

        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;

        //������ģʽѡ���а�����ʮ��·�ڴ�������ģʽһֱֱ�У�ֱ���������߻����ҵ���
        /*if (leave_jump_count >= LEAVE_JUMP_COUNT)
        { //��ֱ�����������㹻��ʱ�������ģʽ
            leave_jump_count = 0;
            (*ppd_now)->l_g_count = 0;
            (*ppd_now)->state = ID_LEAVE_GAR;
        }*/

        fork_dianci_check(induc_value);

        if (pl1->exist && pr1->exist && (lenl1 >= LINE_GET) && (lenr1 >= LINE_GET))
        {
            error[4] = pl1->d - pr1->d;
            (*ppd_now)->state = ID_STRAI; //��Ϊֱ��ģʽ
        }
        else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
        {
            error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
            (*ppd_now)->state = ID_ONEL; //��Ϊ����ģʽ
        }
        else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
        {
            error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
            (*ppd_now)->state = ID_ONER; //��Ϊ�ҵ���ģʽ
        }

        //ֻ�д�ǰ��ֱ�߻���ģʽ�м�⵽�����ߵ�͹���������ڽ������Ķ���ģʽ�н�һ���ж�����
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

            //col_l��col_r�������߰��ߵ�������ֹλ��
            //������ֹλ����Ϊ�����յ㣬������8�ȷ�
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
            //��ÿ���ȷֵ������ҵ���һ���׵㣨������·��͹����ߣ�
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
                //�ҳ����Ǽ������

                //tri_w[]�ǹ̶�Ȩֵ������С���м��
            }

            //˳���������������������
            for (i = 1; i < 9; i++)
            {
                if ((*ppd_now)->tri_i[i] < (*ppd_now)->tri_i[i - 1])
                { //�ҳ�������
                    right_leave_round_point++;
                }

                if ((*ppd_now)->tri_i[i] > (*ppd_now)->tri_i[i - 1])
                { //���������
                    left_leave_round_point++;
                }
            }*/

            //����ĳ����ֵʱ˵���м�ͻ������ԣ�ȷʵ�Ǹ�������״
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
               (*ppd_now)->state = ID_FORK; //��Ϊ����·��ģʽ
               dianci_enter_fork_flag = 0;
               fork_check_flag_1 = 0;
           }

           else if (dianci_right_leave_fork_flag == 1)
           {
               (*ppd_now)->f_count = 0;
               (*ppd_now)->state = ID_R_LEAVE_FORK;//��Ϊ�ҳ�����ģʽ
               dianci_enter_fork_flag = 0;
           }

           else if (dianci_left_leave_fork_flag == 1)
           {
               (*ppd_now)->f_count = 0;
               (*ppd_now)->state = ID_L_LEAVE_FORK;//��Ϊ�������ģʽ
               dianci_enter_fork_flag = 0;
           }

           else if (/*right_leave_round_point >= LEAVE_ROUND_CHECK_LINE*/ right_leave_round_flag == 1)
           {
               right_leave_round_point = 0;
               right_leave_round_flag = 0;
               (*ppd_now)->state = ID_RIGHT_LEAVE_ROUND; //��Ϊ�ҳ���ģʽ
           }

           else if (/*left_leave_round_point >= LEAVE_ROUND_CHECK_LINE*/ left_leave_round_flag == 1)
           {
               left_leave_round_point = 0;
               left_leave_round_flag = 0;
               (*ppd_now)->state = ID_LEFT_LEAVE_ROUND; //��Ϊ�����ģʽ
           }

           else
           {
               right_leave_round_point = 0;
               left_leave_round_point = 0;
               (*ppd_now)->state = ID_LOSE; //ά��ģʽ����
               fork_check_flag_1 = 1;
           }
        }
    }

    else if ((*ppd_last)->state == ID_FORK) //����·��ģʽ
    {
        turn_speed_flag = 1;
        id_getleft(src, *ppd_now);
        id_getright(src, *ppd_now);
        line_getkd(pl1, pointi, pointj);
        line_getkd(pr1, pointi, pointj);
        lenl1 = pl1->begin - pl1->lost + 1;
        lenr1 = pr1->begin - pr1->lost + 1;

        (*ppd_now)->f_count++; //f_count���ڱ�������·��״̬һ��ʱ��

        if ((*ppd_now)->f_dir == F_DIR_R)
        {
            error[4] = error[3] =
                error[2] = error[1] =
                    error[0] = F_R_ERROR; //errorΪ��ʱservoduty��С����ת
            if ((*ppd_now)->f_count >= F_COUNTS)
            {
                (*ppd_now)->f_time++;

                //����ѭ������������
                if ((*ppd_now)->f_time >= 1)
                {
                    (*ppd_now)->f_time = 0;
                    (*ppd_now)->f_dir = F_DIR_L;
                }

                error[4] = error[3] = error[2] =
                    error[1] = error[0] = 0.0;

                //��������·ά�����ں��ٴν��붪��״̬���Ա������ж�����·��
                (*ppd_now)->state = ID_LOSE;
                fork_check_flag = 0;
            }
            else
                (*ppd_now)->state = ID_FORK; //ά��ģʽ����
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
                (*ppd_now)->state = ID_FORK; //ά��ģʽ����
        }
    }

    else if ((*ppd_last)->state == ID_R_LEAVE_FORK) //�ҳ�����
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
                (*ppd_now)->state = ID_STRAI; //��Ϊֱ��ģʽ
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //��Ϊ����ģʽ
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //��Ϊ�ҵ���ģʽ
            }
            else
                (*ppd_now)->state = ID_R_LEAVE_FORK;
        }
        else
            (*ppd_now)->state = ID_R_LEAVE_FORK;
    }

    else if ((*ppd_last)->state == ID_L_LEAVE_FORK) //�������
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
                (*ppd_now)->state = ID_STRAI; //��Ϊֱ��ģʽ
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //��Ϊ����ģʽ
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //��Ϊ�ҵ���ģʽ
            }
            else
                (*ppd_now)->state = ID_L_LEAVE_FORK;
        }
        else
            (*ppd_now)->state = ID_L_LEAVE_FORK;
    }

    else if ((*ppd_last)->state == ID_RIGHT_LEAVE_ROUND) //�ҳ���ģʽ
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
                (*ppd_now)->state = ID_STRAI; //��Ϊֱ��ģʽ
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //��Ϊ����ģʽ
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //��Ϊ�ҵ���ģʽ
            }
            else
                (*ppd_now)->state = ID_RIGHT_LEAVE_ROUND; //����ģʽ����
        }
        else
            (*ppd_now)->state = ID_RIGHT_LEAVE_ROUND; //����ģʽ����
    }

    else if ((*ppd_last)->state == ID_LEFT_LEAVE_ROUND) //�����ģʽ
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
                (*ppd_now)->state = ID_STRAI; //��Ϊֱ��ģʽ
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //��Ϊ����ģʽ
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //��Ϊ�ҵ���ģʽ
            }
            else
                (*ppd_now)->state = ID_LEFT_LEAVE_ROUND; //����ģʽ����
        }
        else
            (*ppd_now)->state = ID_LEFT_LEAVE_ROUND; //����ģʽ����
    }

    //ע�⣺����ʱҲ�ǻᶪ�ߵ�
    else if ((*ppd_last)->state == ID_CLIMB) //����ģʽ
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
                (*ppd_now)->state = ID_STRAI; //��Ϊֱ��ģʽ
            }
            else if (pl1->exist && (lenl1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * (pl1->d - (*ppd_now)->center);
                (*ppd_now)->state = ID_ONEL; //��Ϊ����ģʽ
            }
            else if (pr1->exist && (lenr1 >= LINE_GET * 1.7))
            {
                error[4] = 2.0 * ((*ppd_now)->center - pr1->d);
                (*ppd_now)->state = ID_ONER; //��Ϊ�ҵ���ģʽ
            }
            else
                (*ppd_now)->state = ID_CLIMB; //����ģʽ����
        }
        else
            (*ppd_now)->state = ID_CLIMB; //����ģʽ����
    }
}

void id_getleft(uint8 *src, ImgData *pd)
{
    int16 row, col, sub; //�У��У��±�
    int16 col_info[4];
    uint8 draw_state;
    row = IMG_H - 2;
    col = IMG_W / 2;
    sub = 0;

    //�ҵ���������������е����ұߵ��Ǹ��׵�
    while ((f(src, row, col, IMG_W) == 0) && (col > 0))
        col--;

    col_info[0] = col_info[1] =
        col_info[2] = col_info[3] = col;
    draw_state = 0;

    while (row >= 1)
    {
        row--;
        col += 3;

        //�����ӽ���Զ���ҵ������ÿһ�е����ұߵ��Ǹ��׵�
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
            if (col_info[3] > col_info[1]) //������������
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
    int16 row, col, sub; //�У��У��±�
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
    //TO_DO���������ж�׼����������ƣ�������֮��/֮ǰҪ����������ͬ
    for (row = i; row >= 5; row--)
    { //�ӵױ����������������
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
{ //�м��������������
    int16 row, col;
    row = i / 2;

    for (col = 20; col <= 67; col++)
    {
        //������Ϊ�ڣ��Һڵ�����������������㹻��
        if ((f(src, row, col, IMG_W) == 255 &&
             (f(src, row - 1, col, IMG_W) == 255 || f(src, row + 1, col, IMG_W) == 255) &&
             f(src, row, col + 1, IMG_W)) == 0 &&
            f(src, row, col + 2, IMG_W) == 0 &&
            f(src, row, col + 3, IMG_W) == 0)
        {
            enter_jump_count++;
        }
    }

    //�ݹ۲죬��20-70��ʱ����������׼�������һ����4��5��
    if (enter_jump_count >= 25 && enter_jump_count <= 30)
    {
        /*systick_delay_ms(STM1,300);*/
        //����������ֵ��ֱ��return������enter_gar_time�������
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
    //TO_DO��ͣ���ж�׼�����������
    int row = i, k, count;

    //�ҵ��ױ��е������ߵĵ�һ���׵�
    while (f(src, row, j, IMG_W) == 0 && row > 5)
        row--;

    if (f(src, row, j, IMG_W) == 255 /* && f(src, row - 1, j, IMG_W) == 255*/)
    {
        //����30����
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
            l_angle_flag++; //������ĵ�ÿ��һ�����һ
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
         col_r < 79 && //����4�����У�������ĵ����������һ��
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
         col_r < 70 && //����4�����У�������ĵ����������һ��
         r_angle_flag < 1))
    {
        fork_check_flag_1 = 1;
    }*/
    /* systick_delay_ms(STM1, 333);*/
    l_angle_flag = 0;
    r_angle_flag = 0;
}

void round_check(int16 *value)
{ //TO_DO:��Ҫ���ж���׼�ĳ��������ж�����4000��
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
