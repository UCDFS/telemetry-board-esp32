#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#define WHEEL_SPEED_FL_GPIO GPIO_NUM_36
#define WHEEL_SPEED_FR_GPIO GPIO_NUM_35
#define WHEEL_SPEED_RL_GPIO GPIO_NUM_39
#define WHEEL_SPEED_RR_GPIO GPIO_NUM_34
#define WHEEL_SPEED_GPIO_SEL GPIO_SEL_36 | GPIO_SEL_35 | GPIO_SEL_39 | GPIO_SEL_34
#define WHEEL_SPEED_WAIT_TIME 100

typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR = 1,
    WHEEL_RL = 2,
    WHEEL_RR = 3
} wheel_t;

void wheel_speed_calculation_task();

#endif