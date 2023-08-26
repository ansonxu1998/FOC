//
// Created by Anson
//

#ifndef FOC_FOC_COFING_H
#define FOC_FOC_COFING_H
// 定时器 1
#define PWM_TIM_CLOCK       72000000
#define PWM_TIM_FREQ        10000         //HZ
#define PWM_TIM_PULSE       (PWM_TIM_CLOCK/(2*PWM_TIM_FREQ))
#define PWM_TIM_PULSE_Ts    (PWM_TIM_CLOCK/(PWM_TIM_FREQ))
#define DEAD_TIME           ((uint16_t) 5)
#define PWM_DEAD_TIME       (uint16_t)((unsigned long long)PWM_TIM_CLOCK/2*(unsigned long long)DEAD_TIME/1000000000uL)

#define FOC_PERIOD          0.0001F

// 电机参数
#define RS_PARAMETER     0.59f           // 电阻
#define LS_PARAMETER     0.001f          // 电感
#define FLUX_PARAMETER   0.01150f        // 磁链

// 电流环 PID 参数
#define D_PI_I  1282.8F
#define D_PI_KB  15.0F
#define D_PI_LOW_LIMIT  -24.0F
#define D_PI_P  2.199F
#define D_PI_UP_LIMIT  24.0F
#define Q_PI_I  1282.8F
#define Q_PI_KB  15.0F
#define Q_PI_LOW_LIMIT  -24.0F
#define Q_PI_P  2.199F
#define Q_PI_UP_LIMIT  24.0F

// 速度环 PID 参数
#define SPEED_PID_PERIOD 0.001F
#define SPEED_PI_I  5.0F
#define SPEED_PI_KB  0.015F
#define SPEED_PI_LOW_LIMIT  -5.0F
#define SPEED_PI_P  0.003F
#define SPEED_PI_UP_LIMIT  5.0F

#endif //FOC_FOC_CONFIG_H
