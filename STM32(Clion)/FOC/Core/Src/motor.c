//
// Created by Anson on 2023/8/29.
//
#include "main.h"
#include "motor.h"

uint8_t get_offset_sample_cnt = 0;
uint8_t get_offset_flag = 0;
uint32_t A_offset, B_offset;
float Vbus;
float Ia, Ib, Ic;
float I_ref;
float Iq_ref;
float EKF_Hz;

void get_offset(uint32_t *a_offset, uint32_t *b_offset){
    if (get_offset_sample_cnt < 128){
        *a_offset += ADC1->JDR1;
        *b_offset += ADC1->JDR2;
        get_offset_sample_cnt ++ ;
    } else{
        *a_offset >>= 7;
        *b_offset >>= 7;
        get_offset_sample_cnt = 0;
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        get_offset_flag = 2;
    }
}

void motor_run(void){
    uint32_t Ia_adc, Ib_adc;
    uint32_t Vbus_adc;
    Ia_adc = A_offset - ADC1->JDR1;
    Ib_adc = B_offset - ADC1->JDR2;
    Vbus_adc = ADC1->JDR3;
    Vbus = (float)Vbus_adc * VBUS_CONVERSION_FACTOR;
    Ia = (float)Ia_adc * SAMPLE_CURR_CON_FACTOR;
    Ib = (float)Ib_adc * SAMPLE_CURR_CON_FACTOR;
    Ic = -Ia-Ib;

    if (FOC_Output.EKF[2] > SPEED_LOOP_CLOSE_RAD_S){
        FOC_Input.Id_ref = 0.0f;
        Speed_Fdk = FOC_Output.EKF[2];
        FOC_Input.Iq_ref = Speed_Pid_Out;
    } else{
        FOC_Input.Id_ref = 0.0f;
        FOC_Input.Iq_ref = Iq_ref;
        Speed_PID_G.I_Sum = Iq_ref;
    }
    FOC_Input.theta = FOC_Output.EKF[3];
    FOC_Input.speed_fdk = FOC_Output.EKF[2];

    EKF_Hz = FOC_Output.EKF[2]/(2.0f*PI);
    FOC_Input.Id_ref = 0.0f;
    FOC_Input.Ts = PWM_TIM_PULSE_Ts;
    FOC_Input.Udc = Vbus;
    FOC_Input.Ia = Ia;
    FOC_Input.Ib = Ib;
    FOC_Input.Ic = Ic;
    FOC_Step();
}
