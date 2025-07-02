#include "state_machine.h"
#include <stdio.h>

// 初始化状态机， 在StateMachine_Init中添加参数检查
void StateMachine_Init(StateMachine* sm, 
                      const State* states, uint8_t state_count,
                      const Transition* transitions, uint8_t transition_count,
                      StateID initial_state, void* context, StateMachineLogCallback log_callback) {
    if (!sm || !states || !transitions) return;
    if (initial_state >= state_count) return;
    
    sm->states = states;
       sm->state_count = state_count;
    sm->transitions = transitions;
    sm->transition_count = transition_count;
    sm->current_state = initial_state;
    sm->context = context;
    
    // 调用初始状态的进入函数
    const State* initial = &sm->states[initial_state];
    if (initial->entry) {
        initial->entry(sm, 0, NULL);
    }
}

// 状态转移逻辑单独函数
static bool TryStateTransition(StateMachine* sm, Event event, void* data) {
    for (uint8_t i = 0; i < sm->transition_count; i++) {
        const Transition* transition = &sm->transitions[i];
        
        if (transition->from == sm->current_state && transition->event == event) {
            if (transition->condition && !transition->condition(sm, data)) {
                continue;
            }
            
            const State* current = &sm->states[sm->current_state];
            if (current->exit) {
                current->exit(sm, event, data);
            }
            
            sm->current_state = transition->to;
            const State* new_state = &sm->states[sm->current_state];
            
            if (new_state->entry) {
                new_state->entry(sm, event, data);
            }
            
            return true;
        }
    }
    return false;
}
// 添加状态验证函数
bool StateMachine_Validate(const StateMachine* sm) {
    if (!sm) return false;
    
    // 检查状态ID是否连续且唯一
    for (uint8_t i = 0; i < sm->state_count; i++) {
        if (sm->states[i].id != i) return false;
    }
    
    // 检查转移是否引用有效状态
    for (uint8_t i = 0; i < sm->transition_count; i++) {
        if (sm->transitions[i].from >= sm->state_count ||
            sm->transitions[i].to >= sm->state_count) {
            return false;
        }
    }
    
    return true;
}
// 添加重置函数
void StateMachine_Reset(StateMachine* sm, StateID new_initial_state) {
    if (!sm || new_initial_state >= sm->state_count) return;
    
    // 调用当前状态的退出函数
    const State* current = &sm->states[sm->current_state];
    if (current->exit) {
        current->exit(sm, 0, NULL);
    }
    
    // 设置新初始状态
    sm->current_state = new_initial_state;
    const State* new_state = &sm->states[sm->current_state];
    
    // 调用新状态的进入函数
    if (new_state->entry) {
        new_state->entry(sm, 0, NULL);
    }
}
// 处理事件
bool StateMachine_ProcessEvent(StateMachine* sm, Event event, void* data) {
     if (!sm) return false;
    
    // 查找当前状态
    const State* current = &sm->states[sm->current_state];
    bool handled = false;
    
    // 先调用当前状态的事件处理函数
    if (current->handle_event) {
        current->handle_event(sm, event, data);
        handled = true;
    }
    
    // 尝试状态转移
    if (TryStateTransition(sm, event, data)) {
        handled = true;
    }
    
    return handled;
}

// 添加获取和设置上下文函数
void* StateMachine_GetContext(const StateMachine* sm) {
    return sm ? sm->context : NULL;
}

void StateMachine_SetContext(StateMachine* sm, void* context) {
    if (sm) {
        sm->context = context;
    }
}

// 获取当前状态ID
StateID StateMachine_GetCurrentState(const StateMachine* sm) {
    return sm->current_state;
}
// 添加获取状态信息函数
const State* StateMachine_GetStateInfo(const StateMachine* sm, StateID state) {
    if (!sm || state >= sm->state_count) return NULL;
    return &sm->states[state];
}

// 设置当前状态
void StateMachine_SetCurrentState(StateMachine* sm, StateID state) {
    if (state < sm->state_count) {
        // 执行当前状态的退出函数
        const State* current = &sm->states[sm->current_state];
        if (current->exit) {
            current->exit(sm, 0, NULL);
        }
        
        // 切换到新状态
        sm->current_state = state;
        const State* new_state = &sm->states[sm->current_state];
        
        // 执行新状态的进入函数
        if (new_state->entry) {
            new_state->entry(sm, 0, NULL);
        }
    }
}

// 状态机运行，处理内部逻辑
void StateMachine_Run(StateMachine* sm, uint32_t delta_time) {
    // 查找当前状态
    const State* current = &sm->states[sm->current_state];
    
    // 调用当前状态的事件处理函数，使用特殊事件表示周期性运行
    if (current->handle_event) {
        current->handle_event(sm, 0, (void*)(uintptr_t)delta_time);
    }
    
    // 检查是否有基于时间的状态转移（无事件触发）
    for (uint8_t i = 0; i < sm->transition_count; i++) {
        const Transition* transition = &sm->transitions[i];
        
        if (transition->from == sm->current_state && transition->event == 0) {
            // 检查转移条件（如果有）
            bool can_transition = true;
            if (transition->condition) {
                can_transition = transition->condition(sm, (void*)(uintptr_t)delta_time);
            }
            
            if (can_transition) {
                // 执行当前状态的退出函数
                if (current->exit) {
                    current->exit(sm, 0, NULL);
                }
                
                // 切换到新状态
                sm->current_state = transition->to;
                const State* new_state = &sm->states[sm->current_state];
                
                // 执行新状态的进入函数
                if (new_state->entry) {
                    new_state->entry(sm, 0, NULL);
                }
                
                // 状态转移后不再处理其他转移
                break;
            }
        }
    }
}    