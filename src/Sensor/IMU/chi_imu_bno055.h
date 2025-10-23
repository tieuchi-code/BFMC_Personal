#ifndef _IMU_
#define _IMU_

void CHI_UART_init();
void CHI_UART_Transmit(uint8_t data);
void CHI_UART_send_number(int num);
void CHI_UART_print_log(char *m);
void CHI_UART_send_float(float v);

void I2C1_Master_Init();
void I2C1_WriteReg(uint8_t dev, uint8_t reg, uint8_t data);
void I2C1_ReadMulti(uint8_t dev, uint8_t reg_start, uint8_t* buf, uint8_t len);

uint8_t BNO055_ReadReg(uint8_t reg);
void BNO055_Init(void);
void BNO055_ReadEuler_raw(int16_t* eul_x, int16_t* eul_y, int16_t* eul_z);
uint8_t BNO055_ReadCalibStatus(void);


#endif
