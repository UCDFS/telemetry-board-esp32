#ifndef UCDFS_TELEMETRY_WHEEL_SPEED_H
#define UCDFS_TELEMETRY_WHEEL_SPEED_H

typedef enum
{
	WHEEL_FL = 0,
	WHEEL_FR = 1,
	WHEEL_RL = 2,
	WHEEL_RR = 3
} wheel_t;

void wheel_speed_calculation_task();

#endif //UCDFS_TELEMETRY_WHEEL_SPEED_H