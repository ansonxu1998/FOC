//
// Created by Anson
//

#ifndef FOC_FOC_H
#define FOC_FOC_H

typedef struct {
    float Id_ref;
    float Iq_ref;
    float speed_fdk;
    float theta;
    float Ia;
    float Ib;
    float Ic;
    float Udc;
    float Ts;
} FOC_Input_Def;

typedef struct {
    float Tcmp1;
    float Tcmp2;
    float Tcmp3;
    float EKF[4];
} FOC_Output_Def;

typedef struct {
    float I_alpha;
    float I_beta;
}Alpha_Beta_Current_Def;

typedef struct {
    float I_a;
    float I_b;
    float I_c;
}ABC_Current_Def;

typedef struct {
    float I_q;
    float I_d;
}DQ_Current_Def;

typedef struct {
    float Cos;
    float Sin;
}Trigonometric_Def;

typedef struct {
    float U_aplha;
    float U_beta;
}Alpha_Beat_Voltage_Def;

typedef struct {
    float U_d;
    float U_q;
}DQ_Voltage_Def;

typedef struct {
    float P_Gain;
    float I_Gain;
    float D_Gain;
    float B_Gain;
    float Max_Output;
    float Min_Output;
    float I_Sum;
}Current_PID_Def;

typedef struct
{
    float P_Gain;
    float I_Gain;
    float D_Gain;
    float B_Gain;
    float Max_Output;
    float Min_Output;
    float I_Sum;
}Speed_PID_Def;
extern FOC_Output_Def FOC_Output;
extern FOC_Input_Def FOC_Input;
extern float Speed_Fdk;
extern float Speed_Pid_Out;
extern Speed_PID_Def Speed_PID_G;
extern void FOC_Init(void);
extern void FOC_Step(void);
#endif //FOC_FOC_H
