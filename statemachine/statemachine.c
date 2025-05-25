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

void statemachine_updatestatus(fsm_cb_t *fsm,enum fsm_sig sig)
{
    if(sig == NULL_USE_SING)
    {
        return;
    }
    for(uint8_t i = 0;i<fsm->arr_size;i++)
    {
        if(sig == fsm->arr[i].sig)
        {
            fsm->chState = fsm->arr[i].status;
            break;
        }
    }
}