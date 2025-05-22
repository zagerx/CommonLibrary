#include "statemachine.h"

/**
 * @brief 状态机初始化实现
 */
fsm_rt_t statemachine_init(
    fsm_cb_t *fsm, 
    const char *name,
    fsm_t initial_state,
    void *context)
{
    if (!fsm || !initial_state) {
        return fsm_rt_err;
    }

    fsm->cycle = 0;
    fsm->chState = ENTER;
    fsm->count = 0;
    fsm->name = name;
    fsm->pdata = context;
    fsm->fsm = initial_state;

    return fsm_rt_cpl;
}