#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

// 定义状态ID类型
typedef uint8_t StateID;

// 定义事件类型
typedef uint8_t Event;

// 前置声明状态机结构体
typedef struct StateMachine StateMachine;

// 状态处理函数指针类型
typedef void (*StateHandler)(StateMachine* sm, Event event, void* data);

// 状态转移条件函数指针类型
typedef bool (*TransitionCondition)(StateMachine* sm, void* data);
// 添加日志回调类型
typedef void (*StateMachineLogCallback)(const char* message);

// 状态转移结构体
typedef struct {
    StateID from;       // 源状态
    StateID to;         // 目标状态
    Event event;        // 触发事件
    TransitionCondition condition; // 转移条件函数
} Transition;

// 状态结构体
typedef struct {
    StateID id;                 // 状态ID
    StateHandler entry;         // 进入状态时调用的函数
    StateHandler exit;          // 退出状态时调用的函数
    StateHandler handle_event;  // 处理事件的函数
} State;

// 状态机结构体
typedef struct StateMachine {
    const State* states;        // 状态数组
    uint8_t state_count;        // 状态数量
    const Transition* transitions; // 状态转移数组
    uint8_t transition_count;   // 状态转移数量
    StateID current_state;      // 当前状态
    void* context;              // 状态机上下文数据
    StateMachineLogCallback log_callback;
} StateMachine;

// 初始化状态机
void StateMachine_Init(StateMachine* sm, 
                      const State* states, uint8_t state_count,
                      const Transition* transitions, uint8_t transition_count,
                      StateID initial_state, void* context,StateMachineLogCallback log_callback);

// 处理事件
bool StateMachine_ProcessEvent(StateMachine* sm, Event event, void* data);

// 获取当前状态ID
StateID StateMachine_GetCurrentState(const StateMachine* sm);

// 设置当前状态
void StateMachine_SetCurrentState(StateMachine* sm, StateID state);

// 状态机运行，处理内部逻辑
void StateMachine_Run(StateMachine* sm, uint32_t delta_time);


#endif // STATE_MACHINE_H    