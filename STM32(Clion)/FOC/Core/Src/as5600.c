//
// Created by Anson
//
#include "main.h"
#include "as5600.h"

static float angle_data_prev;
static float full_rotations;

void AS5600_WriteReg(uint8_t reg_addr, uint8_t reg_dat) {
    uint8_t sendbuf[2];
    sendbuf[0] = reg_addr;
    sendbuf[1] = reg_dat;
    HAL_I2C_Master_Transmit(&hi2c1, AS5600_ADDRESS, sendbuf, 2, 10);
}

void AS5600_ReadDate(uint8_t reg_addr, uint8_t *reg_data, uint8_t num) {
    HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, num, 50);
}

void AS5600_Init(void) {
    full_rotations = 0;
    angle_data_prev = AS5600_ReadRawData();
}

uint16_t AS5600_ReadRawData(void) {
    uint16_t raw_data;
    uint8_t buf[2] = {0};
    AS5600_ReadDate(AS5600_RAW_ANGLE_REGISTER, buf, 2);
    raw_data = ((uint16_t) buf[0] << 8) | (uint16_t) buf[1];
    return raw_data;
}

float AS5600_ReadAngle(void){
    uint16_t raw_data;
    raw_data = AS5600_ReadRawData();
    return (float) raw_data * 0.08789f * _PI / 180;
}

float AS5600_ReadAngle_Accumulative(void) {
    float val = AS5600_ReadAngle();
    float d_angle = val - angle_data_prev;
    if (abs(d_angle) > (0.8f * _2PI)) full_rotations += (d_angle > 0) ? -1 : 1;
    angle_data_prev = val;
    return (float) full_rotations * _2PI + angle_data_prev;
}

