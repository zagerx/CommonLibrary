/*
 * 轨迹规划算法头文件
 * 包含S型和线性轨迹规划接口定义
 * 
 * Copyright (c) 2023 DeepSeek Company
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef S_TRAJECTORY_PLANNING_H
#define S_TRAJECTORY_PLANNING_H

#include <stdint.h>
#include <stdbool.h>

/* 轨迹规划状态枚举 */
typedef enum {
    TRAJ_STATE_IDLE = 0,
    TRAJ_STATE_RISING,
    TRAJ_STATE_STEP1,
    TRAJ_STATE_STEP2,
    TRAJ_STATE_STEP3,
    TRAJ_STATE_FAIL,
    TRAJ_STATE_SUCCESS,
    TRAJ_STATE_ZERO_POINT,
    TRAJ_STATE_OVER,
    TRAJ_STATE_WAIT
} trajectory_state_t;

/* S型轨迹规划结构体 */
typedef struct {
    uint16_t cout;          /* 当前步数计数 */
    uint16_t tau;           /* 时间参数 */
    uint16_t Ts_Max;            /* 最大时间周期 */
    uint16_t Ts[4];         /* 各阶段时间分配 */
    trajectory_state_t actor_state ; /* 执行状态 */
     
    float Ja;               /* 加加速度 */
    float V[4];             /* 各阶段结束速度 */
    float V0;               /* 初始速度 */
    float cur_output;       /* 当前输出值 */
    float last_target;      /* 上次目标值 */
    float max_acc;          /* 最大加速度 */
    float max_ja;
} s_in_t;

/* API接口 */
float s_type_interpolation(void *object, float new_target);

#endif /* TRAJECTORY_PLANNING_H */