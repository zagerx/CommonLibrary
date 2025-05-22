/**
 * @file statemachine.h
 * @brief Finite State Machine (FSM) framework
 *
 * Provides:
 * - State machine return codes
 * - Basic state machine control block
 * - State transition macros
 *
 * Copyright (c) 2023 Your Company
 * SPDX-License-Identifier: Apache-2.0
 */

 #ifndef _STATEMACHINE_H
 #define _STATEMACHINE_H
 
 /**
  * @enum fsm_rt_t
  * @brief State machine return codes
  */
 typedef enum {
     fsm_rt_err          = -1,    ///< FSM error (check error code from other interface)
     fsm_rt_cpl          = 0,     ///< FSM completed successfully
     fsm_rt_on_going     = 1,     ///< FSM operation in progress
     fsm_rt_wait_for_obj = 2,     ///< FSM waiting for resource/object
     fsm_rt_asyn         = 3,     ///< FSM in asynchronous mode (check status later)
 } fsm_rt_t;
 
 /**
  * @enum Standard states
  * @brief Common state machine states
  */
 enum {
     ENTER = 0,    ///< Entry state handler
     EXIT,         ///< Exit state handler      
     USER          ///< First available user-defined state
 };
 
 /* Forward declarations */
 typedef struct fsm_cb fsm_cb_t;
 typedef fsm_rt_t (*fsm_t)(fsm_cb_t*);
 
 /**
  * @struct fsm_cb
  * @brief State machine control block
  */
 struct fsm_cb {
     unsigned short cycle;  ///< Execution cycle count
     unsigned char chState; ///< Current state
     unsigned int count;    ///< General purpose counter
     const char* name;      ///< State machine name (for debugging)
     void* pdata;          ///< User data pointer
     fsm_t fsm;            ///< Current state handler function
 };
 
 /**
  * @brief Execute current state handler
  * @param me_ State machine control block pointer
  */
 #define DISPATCH_FSM(me_) ((fsm_t)(me_)->fsm)((me_))
 
 /**
  * @brief Transition between states
  * @param me State machine control block
  * @param targe Target state handler function
  *
  * Handles proper state transitions by:
  * 1. Executing exit handler for current state
  * 2. Changing to new state handler
  * 3. Executing entry handler for new state
  */
 #define TRAN_STATE(me,targe) do {                      \
                                (me)->chState = EXIT;      \
                                (me)->fsm(me);             \
                                (me)->fsm = (fsm_t)(targe);\
                                (me)->chState = ENTER;     \
                                (me)->fsm(me);             \
                             } while(0)
 
 /* Function prototypes */
fsm_rt_t statemachine_init(fsm_cb_t *fsm, 
                  const char *name,
                  fsm_t initial_state,
                  void *context);
 #endif /* _STATEMACHINE_H */