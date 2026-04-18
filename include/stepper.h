#ifndef STEPPER_H
#define STEPPER_H

#include "driver/gpio.h"

// L298N IN pins drive the 4-step coil sequence
#define IN1_PIN GPIO_NUM_0
#define IN2_PIN GPIO_NUM_1
#define IN3_PIN GPIO_NUM_2
#define IN4_PIN GPIO_NUM_21

// ENA/ENB carry PWM to limit coil current and reduce L298N heating.
// Remove the ENA/ENB jumpers and wire these pins instead.
#define ENA_PIN GPIO_NUM_22
#define ENB_PIN GPIO_NUM_23

// 0–1023 (10-bit). 700 ≈ 68% duty. Lower = cooler / less torque.
#define ENABLE_DUTY 700

#define STEP_PERIOD_MS 10
#define STEPS_PER_REV  200
#define STEPS_PER_90   (STEPS_PER_REV / 4)

#define CW  (-1)
#define CCW (+1)

extern float angle;

void initStepper();
void stepperTask();

#endif
