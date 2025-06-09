#include "s_trajectory_planning.h"
#include "debuglog.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "string.h"
#include "stdlib.h"
#define EPSILON 0.01f // 用于浮点数比较的容差
enum
{
    RISING = 0,
    T1 = 1,
    T2 = 2,
    T3 = 3,
    FALLING = 8,
    IDLE = 9,
    INIT = 10,
    FAIL = 11,
    S_SUCCESS = 12,
};
/*==========================================================================================
 * @brief        S型轨迹规划
 * @FuncName
 * @param        object
 * @param        new_targe
 * @version      0.1
--------------------------------------------------------------------------------------------*/
static void s_shape_path_planning(void *object, float new_targe)
{
    enum
    {
        INIT,
        STEP1,
        STEP2,
        STEP3,
        PLAN_FAIL,
        PLAN_SUCESS,
    };
    s_in_t *s = (s_in_t *)object;
    float diff;
    float v0, v1, v2, v3;
    float temp_delte;
    float temp_ja;
    uint16_t T2;
    uint16_t T1;
    int16_t plan_state = INIT;
    /*规划更新Ja*/
    plan_state = INIT;
    while (1)
    {
        switch (plan_state)
        {
        case INIT:
            diff = new_targe - s->last_target;
            s->V0 = s->cur_output;
            v0 = s->cur_output;
            temp_ja = s->max_ja;
            plan_state = STEP1;
            break;
        case STEP1://更新规划时间
            T2 = s->Ts_Max / 2;
            T1 = (s->Ts_Max - T2) / 2;
            if (diff > EPSILON)
            {
                s->Ts[1] = s->Ts[3] = T1; // 加速/减速阶段 时间需要保持一致
                s->Ts[2] = T2;
            }
            plan_state = STEP2;
            break;
        case STEP2:
            if (temp_ja < 0.0f) // 规划失败
            {
                plan_state = PLAN_FAIL;
            }
            temp_ja -= 0.01f;
            s->Ja = (diff > EPSILON) ? temp_ja : -temp_ja;
            v1 = v0 + (0.5f) * (s->Ja) * (T1) * (T1);
            v2 = v1 + (T1) * (s->Ja) * (T2);
            v3 = v2 + T1 * s->Ja * T1 - 0.5f * s->Ja * T1 * T1;
            temp_delte = (diff > EPSILON) ? (v3 - new_targe) : -(v3 - new_targe);
            if (temp_delte < EPSILON) // 规划成功
            {
                plan_state = PLAN_SUCESS; // 规划成功
                break;
            }
            break;
        case PLAN_FAIL:
            return;
            break;
        case PLAN_SUCESS:
            s->cout = 0;
            s->tau = 0;
            s->actor_state  = RISING;
            plan_state = INIT; // 规划成功
            return;
            break;
        default:
            break;
        }
    }
    return;
}
/*==========================================================================================
 * @brief        S型插补(指数函数)
 * @FuncName     s_type_interpolation
 * @param        object
 * @param        new_targe
 * @return       float
 * @version      0.1
 *
 * --------------------------------------------------------------------------------------------*/
float s_type_interpolation(void *object, float new_targe)
{
    float out_val;
    s_in_t *s = (s_in_t *)object;

    if (s->last_target != new_targe) // 路径规划
    {
        s_shape_path_planning(s, (new_targe));
        s->last_target = new_targe;
    }

    switch (s->actor_state )
    {
    case RISING:
    case FALLING:
        s->actor_state  = T1;
    case T1:
        s->V[1] = s->V0 + (0.5f) * (s->Ja) * (s->tau) * (s->tau);
        out_val = s->V[1];
        if (s->cout++ < s->Ts[1]) // 第一阶段未完成
        {
            s->tau++;
            break;
        }
        s->cout = 0;
        s->tau = 0;
        s->actor_state  = T2;
        break;

    case T2:
        s->V[2] = s->V[1] + (s->Ts[1]) * (s->Ja) * (s->tau);
        ;
        out_val = s->V[2];
        if (s->cout++ < s->Ts[2])
        {
            s->tau++;
            break;
        }
        s->cout = 0;
        s->tau = 0;
        s->actor_state  = T3;
        break;
    case T3:
        s->V[3] = s->V[2] + s->Ts[1] * s->Ja * s->tau - 0.5f * s->Ja * s->tau * s->tau;
        out_val = s->V[3];
        if (s->cout++ < s->Ts[3])
        {
            s->tau++;
            break;
        }
        s->cout = 0;
        s->tau = 0;
        s->actor_state  = IDLE;
        break;
    case IDLE:
        return;
        break;

    default:
        break;
    }
    s->cur_output = out_val;
    return out_val;
}
void s_type_interpolation_init(void *object, float a_max,float Ja_max, uint16_t T_max)
{
    s_in_t *s = (s_in_t *)object;
    s->actor_state  = IDLE;
    s->max_acc = a_max;
    s->max_ja = Ja_max;
    s->cur_output = 0.0f;
    s->Ts_Max = T_max;
}
void s_type_interpolation_deinit(void *object)
{
    memset(object, 0, sizeof(s_in_t));
}

#include "taskmodule.h"
#if 1
void test(void)
{
    enum
    {
        INIT,
        STEP1,
        STEP2,
        STEP3,
        STEP4,
        STEP5,
    };
    static s_in_t test_s_val;
    static float target = 0.0f;
    static uint8_t flag = 0;
    static uint8_t cnt = 0;
    static uint8_t status_ = INIT;
    switch (status_)
    {
    case INIT:
        /* code */
        target = 100.0f;
        s_type_interpolation_init(&test_s_val,20.0f, 20.0f, 40);
        status_ = STEP1;
        break;
    case STEP1:
        if (cnt++ > 200)
        {
            cnt = 0;
            target += 100.0f;
            if (target > 300.0f)
            {
                status_ = STEP2;
                break;
            }
        }
        break;
    case STEP2:
        if (cnt++ > 200)
        {
            cnt = 0;
            target -= 100.0f;
            if (target < -100.0f)
            {
                status_ = STEP3;
                break;
            }
        }
        break;
    case STEP3:
        if (cnt++ > 200)
        {
            cnt = 0;
            target = 800.0f;
            status_ = STEP4;
        }
        break;
    case STEP4:
        if (cnt++ > 200)
        {
            cnt = 0;
            target = -800.0f;
            status_ = STEP5;
        }
        break;
    default:
        break;
    }
    float output;
    output = s_type_interpolation(&test_s_val, target);
    return 0;
}
board_task(test)
#endif

