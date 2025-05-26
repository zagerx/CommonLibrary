#include "statemachine.h"
#include <stdint.h>

/**
 * @brief 状态机初始化实现
 */
fsm_rt_t statemachine_init(
    fsm_cb_t *fsm, 
    const char *name,
    fsm_t initial_state,
    void *context,struct state_transition_map* arr,int8_t arr_size)
{
    if (!fsm || !initial_state) {
        return fsm_rt_err;
    }

    fsm->cycle = 0;
    fsm->chState = ENTER;
    fsm->count = 0;
    fsm->name = name;
    fsm->p1 = context;
    fsm->fsm = initial_state;
    fsm->transition_table = arr;
    fsm->transition_table_size = arr_size;
    fsm->sig = NULL_USE_SING;
    return fsm_rt_cpl;
}

void statemachine_updatestatus(fsm_cb_t *fsm,enum fsm_signal sig)
{
    if(sig == NULL_USE_SING)
    {
        return;
    }
    for(uint8_t i = 0;i<fsm->transition_table_size;i++)
    {
        if(sig == fsm->transition_table[i].signal)
        {
            fsm->chState = fsm->transition_table[i].target_state;
            break;
        }
    }
}

void statemachine_setsig(fsm_cb_t *fsm,enum fsm_signal sig)
{
    fsm->sig = sig;
}
