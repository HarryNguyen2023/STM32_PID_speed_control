#include "PID_motor.h"
#include "PID_motor_cfg.h"
#include <stdlib.h>

// Define some PID variables
int Kp = 20;
int Ki = 0;
int Kd = 12;
int Ko = 50;
uint16_t time_frame = 5;    // Time frame to milisecond

// ----------------------------------------------------- Static functions hidden from users ----------------------------------------------
// Function prototypes
static void resetPID(PID_motor* motor);
static void outputPID(PID_motor* motor);
static void dutyCycleUpdate(uint16_t duty_cycle, PID_motor* motor);

// Function to reset the PID value of the motor when it is not moving
static void resetPID(PID_motor* motor)
{
    motor->current_encoder = readEncoder(motor);
    motor->prev_encoder = motor->current_encoder;
    motor->integral_error = 0;
    motor->output = 0;
    motor->prev_encoder_feedback = 0;
    motor->targetPulsePerFrame = 0.0;
}

// Function to get the output value of the PID speed controller
static void outputPID(PID_motor* motor)
{
    int16_t feedback, output;
    double error;
    // Get number of the encoder pulse in the last time frame
    feedback = motor->current_encoder - motor->prev_encoder;
    // Get the error of the number of encoder per time frame
    error = motor->targetPulsePerFrame - feedback;
    // Get the output of the PID controller with the new formula to avoid derivative kick as well as accumulation error when updating PID parameters
    output = (Kp * error - Kd * (feedback - motor->prev_encoder_feedback) + motor->integral_error) / Ko;
    motor->integral_error += Ki * error;
    // Update the parameters
    motor->prev_encoder = motor->current_encoder;
    motor->prev_encoder_feedback = feedback;
    // Limit the ouput velocity of the motor
    output += motor->output;
    if(output > motor->MAX_PWM)
        output = motor->MAX_PWM;
    else if(output < - motor->MAX_PWM)
        output = - motor->MAX_PWM;
    motor->output = output;
}

// Function to update the PWM duty cycle of the motor
static void dutyCycleUpdate(uint16_t duty_cycle, PID_motor* motor)
{
    switch (motor->pwm_channel)
    {
        case PWM_CHANNEL_1:
            motor->pwm_tim->CCR1 = duty_cycle;
            break;
        case PWM_CHANNEL_2:
            motor->pwm_tim->CCR2 = duty_cycle;
            break;
        case PWM_CHANNEL_3:
            motor->pwm_tim->CCR3 = duty_cycle;
            break;
        case PWM_CHANNEL_4:
            motor->pwm_tim->CCR4 = duty_cycle;
            break;
    default:
        break;
    }
}

// -------------------------------------------------------- General function used by users -----------------------------------------------

// Function to initiate the motor GPIO pins
void motorInit(PID_motor motor)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // Initiate the GPIO pins of the motor
    for(int i = 0; i < 2; ++i)
    {
        HAL_GPIO_WritePin(motor.motor_ports[i], motor.motor_pins[i], 0);
        if(motor.motor_ports[i] == GPIOA)
            __HAL_RCC_GPIOA_CLK_ENABLE();
        else if (motor.motor_ports[i] == GPIOB)
            __HAL_RCC_GPIOB_CLK_ENABLE();
        else if (motor.motor_ports[i] == GPIOC)
            __HAL_RCC_GPIOC_CLK_ENABLE();
        else if (motor.motor_ports[i] == GPIOD)
            __HAL_RCC_GPIOD_CLK_ENABLE();
        else if (motor.motor_ports[i] == GPIOE)
            __HAL_RCC_GPIOE_CLK_ENABLE();
        GPIO_InitStruct.Pin = motor.motor_pins[i];
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(motor.motor_ports[i], &GPIO_InitStruct);
    }
}

// Function to get the encoder value of the motor
int32_t readEncoder(PID_motor* motor)
{
    return ((int32_t)motor->encoder_tim->CNT) / 4;
}

// Function to brake the motor immediately
void motorBrake(PID_motor* motor)
{
    HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], 0);
    HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], 0);
}

// Function to update the PID parameters
void updatePID(int* pid_params)
{
    Kp = pid_params[0];
    Ki = pid_params[1];
    Kd = pid_params[2];
    Ko = pid_params[3];
}

// Function to handle the speed input of the PID controller
void inputSpeedHandling(TIM_HandleTypeDef* htim, PID_motor* motor, int speed)
{
    // Rescale the input rpm speed
    if(speed > motor->MAX_INPUT_SPEED)
        speed = motor->MAX_INPUT_SPEED;
    else if(speed < - motor->MAX_INPUT_SPEED)
        speed = - motor->MAX_INPUT_SPEED;
    // Check if command the motor to stop
    if(speed == 0)
    {
        motorBrake(motor);
        motor->moving = 0;
        // Stop the timer 
        HAL_TIM_Base_Stop_IT(htim);
        // Reaset the PID value
        resetPID(motor);
        return;
    }
    // Check whether the motor is already moving
    if(! motor->moving)
    {
        motor->moving = 1;
        // Start the timer for controlling the motor speed
        HAL_TIM_Base_Start_IT(htim);
    }
    // Convert the desired speed to pulse per frame and input to the motor
    motor->targetPulsePerFrame = (speed * motor->encoder_rev * 1.0) * time_frame / 60000.0;
}

// Function to control the speed of the motor by PID algorithm
void speedControlPID(PID_motor* motor)
{
    // Update the motor encoder value
    motor->current_encoder = readEncoder(motor);
    // Update the PID output of the controller
    outputPID(motor);

    // Get the direction of the motor
    uint8_t direction;
    if(motor->output >= 0)
        direction = 0;
    else
        direction = 1;

    // Get the absolute value of the motor
    uint16_t pwm_dutycycle = abs(motor->output);
    if(pwm_dutycycle < motor->DEAD_BAND)
        pwm_dutycycle = motor->DEAD_BAND;

    // Control the direction of the motor
    if(direction == 0)
    {
        HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], 1);
        HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], 0);
    }
    else if(direction == 1)
    {
        HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], 0);
        HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], 1);
    }
    // Feed the value of the PWM duty cycle
    dutyCycleUpdate(pwm_dutycycle, motor);
}

