#ifndef PID_MOTOR_H
#define PID_MOTOR_H

#include "stm32f1xx_hal.h"

// Define some variables
extern int Kp;
extern int Ki;
extern int Kd;
extern int Ko;
extern uint16_t time_frame;

typedef enum
{
    PWM_CHANNEL_1,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
    PWM_CHANNEL_4
}PWM_CHANNEL;

// Structure to contains the control information of the motors
typedef struct
{
    // Pin for control the motor directions
    GPIO_TypeDef* motor_ports[2];
    uint16_t motor_pins[2];
    // Timer module for controlling the encoder
    TIM_TypeDef* encoder_tim;
    TIM_TypeDef* pwm_tim;
    PWM_CHANNEL pwm_channel;
    
    // Encoder resolution of the motor
    uint16_t encoder_rev;

    // Motor speed specification
    uint16_t MAX_PWM;
    uint16_t MAX_INPUT_SPEED;
    uint8_t DEAD_BAND;

    // PID controller parameters
    double targetPulsePerFrame;
    int32_t current_encoder;
    int32_t prev_encoder;
    uint8_t prev_encoder_feedback;
    uint16_t integral_error;
    int16_t output;
    uint8_t moving;
}PID_motor;

// Function prototypes
void motorInit(PID_motor motor);
int32_t readEncoder(PID_motor* motor);
void speedControlPID(PID_motor* motor);
void positionControlPID(PID_motor* motor);
void motorBrake(PID_motor* motor);
void updatePID(int* pid_params);
void inputSpeedHandling(TIM_HandleTypeDef* htim, PID_motor* motor, int speed);

#endif