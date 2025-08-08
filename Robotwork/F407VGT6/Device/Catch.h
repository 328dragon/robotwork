#include "stm32f4xx_hal.h"
#include "tcs230.h"
#include "tim.h"

extern int catch_finish_flag;
void Sort(enum color_enum color);

enum color_enum Catch();

void Drop(enum color_enum color);