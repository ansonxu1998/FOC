//
// Created by Anson
//
#include "foc.h"
#include "main.h"

// 外部函数
extern void EKF_Outputs(float *y, const float *xD);

extern void EKF_Update(const float *u, float *xD);

extern void EKF_Start(float *xD);

// 全局变量
FOC_Output_Def FOC_Output;
FOC_Input_Def FOC_Input;
ABC_Current_Def ABC_Current_G;
Alpha_Beta_Current_Def Alpha_Beta_Current_G;
Trigonometric_Def Trigonometric_G;
DQ_Current_Def DQ_Current_G;
DQ_Voltage_Def DQ_Voltage_G;
Current_PID_Def Current_PID_D_G;
Current_PID_Def Current_PID_Q_G;
Speed_PID_Def Speed_PID_G;
Alpha_Beat_Voltage_Def Alpha_Beat_Voltage_G;
float EKF_Input[4];
float EKF_Status[4];

void ClarkTransform(ABC_Current_Def ABC_Current, Alpha_Beta_Current_Def *Alpha_Beta_Current) {
    Alpha_Beta_Current->I_alpha = ABC_Current.I_a;
    Alpha_Beta_Current->I_beta = (ABC_Current.I_a + 2 * ABC_Current.I_b) * 1.73205080756887F / 3.0F;
}

// 计算角度的三角函数
void Trigonometric_function(float theta, Trigonometric_Def *trigonometric) {
    trigonometric->Cos = arm_cos_f32(theta);
    trigonometric->Sin = arm_sin_f32(theta);
}

// Park 变换
void
ParkTransform(Alpha_Beta_Current_Def Alpha_Beta_Current, Trigonometric_Def Trigonometric, DQ_Current_Def *DQ_Current) {
    DQ_Current->I_d = Alpha_Beta_Current.I_alpha * Trigonometric.Cos + Alpha_Beta_Current.I_beta * Trigonometric.Sin;
    DQ_Current->I_q = -Alpha_Beta_Current.I_alpha * Trigonometric.Sin + Alpha_Beta_Current.I_beta * Trigonometric.Cos;
}

// 反 Park 变换
void InverseParkTransform(DQ_Voltage_Def DQ_Voltage, Trigonometric_Def Trigonometric,
                          Alpha_Beat_Voltage_Def *Alpha_Beat_Voltage) {
    Alpha_Beat_Voltage->U_aplha = DQ_Voltage.U_d * Trigonometric.Cos - DQ_Voltage.U_q * Trigonometric.Sin;
    Alpha_Beat_Voltage->U_beta = DQ_Voltage.U_d * Trigonometric.Sin + DQ_Voltage.U_q * Trigonometric.Cos;
}

void SVPWM(Alpha_Beat_Voltage_Def Alpha_Beat_Voltage, float Udc, float Ts) {
    uint8_t N = 0;
    float Tcmp1, Tcmp2, Tcmp3, Ta, Tb, Tc, T1, T2, T3, Tx, Ty, f_temp;
    if (Alpha_Beat_Voltage.U_beta > 0.0F) N = 1;
    if ((1.73205078F * Alpha_Beat_Voltage.U_aplha - Alpha_Beat_Voltage.U_beta) * 0.5F > 0.0F) N = N + 2;
    if ((-1.73205078F * Alpha_Beat_Voltage.U_aplha - Alpha_Beat_Voltage.U_beta) * 0.5F > 0.0F) N = N + 4;

    T1 = (1.73205078F * Alpha_Beat_Voltage.U_beta) * (Ts / Udc);
    T2 = (1.5F * Alpha_Beat_Voltage.U_aplha + 0.866025388F * Alpha_Beat_Voltage.U_beta) * (Ts / Udc);
    T3 = (-1.5F * Alpha_Beat_Voltage.U_aplha + 0.866025388F * Alpha_Beat_Voltage.U_beta) * (Ts / Udc);

    // Tx, Ty 矢量作用时间计算
    switch (N) {
        case 1: // Sector = 2
            Tx = T3;
            Ty = T2;
            break;
        case 2: // Sector = 6
            Tx = T2;
            Ty = -T1;
            break;
        case 3: // Sector = 1
            Tx = -T3;
            Ty = T1;
            break;
        case 4: // Sector = 4
            Tx = -T1;
            Ty = T3;
            break;
        case 5: // Sector = 3
            Tx = T1;
            Ty = -T2;
            break;
        default: // Sector = 5
            Tx = -T2;
            Ty = -T3;
            break;
    }

    f_temp = Tx + Ty;
    // 判断是否发生过调制
    if (f_temp > Ts) {
        Tx /= f_temp;
        Ty /= (Tx + Ty);
    }

    Ta = (Ts - Tx - Ty) / 4.0F;
    Tb = (Ts + Tx + Ty) / 4.0F;
    Tc = (Ts + Tx + Ty) / 4.0F;

    // 计算定时器比较值
    switch (N) {
        case 1: // Sector = 2
            Tcmp1 = Tb;
            Tcmp2 = Ta;
            Tcmp3 = Tc;
            break;
        case 2: // Sector = 6
            Tcmp1 = Ta;
            Tcmp2 = Tc;
            Tcmp3 = Tb;
            break;
        case 3: // Sector = 1
            Tcmp1 = Ta;
            Tcmp2 = Tb;
            Tcmp3 = Tc;
            break;
        case 4: // Sector = 4
            Tcmp1 = Tc;
            Tcmp2 = Tb;
            Tcmp3 = Ta;
            break;
        case 5: // Sector = 3
            Tcmp1 = Tc;
            Tcmp2 = Ta;
            Tcmp3 = Tb;
            break;
        default: // Sector = 5
            Tcmp1 = Tb;
            Tcmp2 = Tc;
            Tcmp3 = Ta;
            break;
    }
    FOC_Output.Tcmp1 = Tcmp1;
    FOC_Output.Tcmp2 = Tcmp2;
    FOC_Output.Tcmp3 = Tcmp3;
}

void Current_PID(float ref_current, float feadback_current, float *voltage, Current_PID_Def *Current_PID) {
    float error, temp;
    error = ref_current - feadback_current;
    temp = Current_PID->P_Gain * error + Current_PID->I_Sum;
    if (temp > Current_PID->Max_Output) {
        *voltage = Current_PID->Max_Output;
    } else if (temp < Current_PID->Min_Output) {
        *voltage = Current_PID->Min_Output;
    } else {
        *voltage = temp;
    }
    Current_PID->I_Sum += ((*voltage - temp) * Current_PID->B_Gain + Current_PID->I_Gain * error) * FOC_PERIOD;
}

void Speed_PID(float ref_speed, float feedback_sped, float *ref_current, Speed_PID_Def *Speed_PID) {
    float error;
    float temp;

    error = 6.28318548F * ref_speed - feedback_sped;

    temp = (error + Speed_PID->I_Sum) * Speed_PID->P_Gain;


    if (temp > Speed_PID->Max_Output) {
        *ref_current = Speed_PID->Max_Output;
    } else if (temp < Speed_PID->Min_Output) {
        *ref_current = Speed_PID->Min_Output;
    } else {
        *ref_current = temp;
    }
    Speed_PID->I_Sum += ((*ref_current - temp) * Speed_PID->B_Gain + Speed_PID->I_Gain * error) * SPEED_PID_PERIOD;
}

void FOC_Step(void) {
    ABC_Current_G.I_a = FOC_Input.Ia;
    ABC_Current_G.I_b = FOC_Input.Ib;
    ABC_Current_G.I_c = FOC_Input.Ic;

    // Clark 变换
    ClarkTransform(ABC_Current_G, &Alpha_Beta_Current_G);
    // 计算角度的三角函数值
    Trigonometric_function(FOC_Input.theta, &Trigonometric_G);
    // Park 变换
    ParkTransform(Alpha_Beta_Current_G, Trigonometric_G, &DQ_Current_G);
    // D 轴电流环 PID
    Current_PID(FOC_Input.Id_ref, DQ_Current_G.I_d, &DQ_Voltage_G.U_d, &Current_PID_D_G);
    // Q 轴电流环 PID
    Current_PID(FOC_Input.Iq_ref, DQ_Current_G.I_q, &DQ_Voltage_G.U_q, &Current_PID_Q_G);
    // 反 Park 变换
    InverseParkTransform(DQ_Voltage_G, Trigonometric_G, &Alpha_Beat_Voltage_G);
    // 扩展卡尔曼滤波输入
    EKF_Input[0] = Alpha_Beat_Voltage_G.U_aplha;
    EKF_Input[1] = Alpha_Beat_Voltage_G.U_beta;
    EKF_Input[2] = Alpha_Beta_Current_G.I_alpha;
    EKF_Input[3] = Alpha_Beta_Current_G.I_beta;
    // 扩展卡尔曼滤输出
    EKF_Outputs(&FOC_Output.EKF[0], &EKF_Status[0]);
    //  SVPWM 输出
    SVPWM(Alpha_Beat_Voltage_G, FOC_Input.Udc, FOC_Input.Ts);
    // 扩展卡尔曼滤波状态更新
    EKF_Update(&EKF_Input[0], &EKF_Status[0]);
}

void FOC_Init(void) {
    // PID 参数
    Current_PID_D_G.P_Gain = D_PI_P;
    Current_PID_D_G.I_Gain = D_PI_I;
    Current_PID_D_G.B_Gain = D_PI_KB;
    Current_PID_D_G.Max_Output = D_PI_UP_LIMIT;
    Current_PID_D_G.Min_Output = D_PI_LOW_LIMIT;
    Current_PID_D_G.I_Sum = 0.0F;

    Current_PID_Q_G.P_Gain = Q_PI_P;
    Current_PID_Q_G.I_Gain = Q_PI_I;
    Current_PID_Q_G.B_Gain = Q_PI_KB;
    Current_PID_Q_G.Max_Output = Q_PI_UP_LIMIT;
    Current_PID_Q_G.Min_Output = Q_PI_LOW_LIMIT;
    Current_PID_Q_G.I_Sum = 0.0F;

    Speed_PID_G.P_Gain = SPEED_PI_P;
    Speed_PID_G.I_Gain = SPEED_PI_I;
    Speed_PID_G.B_Gain = SPEED_PI_KB;
    Speed_PID_G.Max_Output = SPEED_PI_UP_LIMIT;
    Speed_PID_G.Min_Output = SPEED_PI_LOW_LIMIT;
    Speed_PID_G.I_Sum = 0.0f;
    // 启动 EKF
    EKF_Start(&EKF_Status[0]);
}