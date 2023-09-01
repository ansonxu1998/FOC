//
// Created by Anson
//

#ifndef FOC_AS5600_H
#define FOC_AS5600_H

#define AS5600_ADDRESS              0x36 << 1
#define AS5600_RAW_ANGLE_REGISTER   0x0c
#define abs(x)                      ((x)>0?(x):(-x))
#define _2PI                        6.28318530718f
#define _PI                         3.14159265358f

void AS5600_Init(void);
void AS5600_WriteReg(uint8_t reg_addr, uint8_t reg_dat);
void AS5600_ReadDate(uint8_t reg_addr, uint8_t* reg_data, uint8_t num);
uint16_t AS5600_ReadRawData(void);
float AS5600_ReadAngle(void);
float AS5600_ReadAngle_Accumulative(void);

#endif //FOC_AS5600_H
