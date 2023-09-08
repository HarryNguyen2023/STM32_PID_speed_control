#ifndef PID_MOTOR_H
#define PID_MOTOR_H

#include "stm32f1xx_hal.h"

#define MAX_PWM 499
// The maximum rpm speed of the motor
#define MAX_INPUT_SPEED 250 

// Define some variables
extern float Kp;
extern float Ki;
extern float Kd;
extern uint16_t time_frame;

// Structure to contains the control information of the motors
typedef struct
{
    // Pin for control the motor directions
    GPIO_TypeDef motor_ports[2];
    uint16_t motor_pins[2];
    // Timer module for controlling the encoder
    TIM_TypeDef encoder_tim;
    TIM_TypeDef pwm_tim;
    double targetPulsePerFrame;
    long current_encoder;
    long prev_encoder;
    uint8_t prev_input_speed;
    uint16_t integral_error;
    long output;
}PID_motor;

// Function prototypes
void motorInit(void);
long getEncoder(PID_motor motor);
void speedControlPID(PID_motor* motor);
void positionControlPID(PID_motor* motor);
void motorBrake(PID_motor motor);

#endif