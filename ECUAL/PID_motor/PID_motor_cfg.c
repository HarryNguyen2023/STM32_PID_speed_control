#include "PID_motor.h"

PID_motor motor1 = 
{
    {GPIOA, GPIOA},
    {GPIO_PIN_1, GPIO_PIN_2},
    TIM3,
    TIM4,
    PWM_CHANNEL_1,
    374,
    499,
    250,
    80,
    0.0,
    0,
    0,
    0,
    0,
    0,
    0
};