/*
 * bno055.h
 *
 *  Created on: Dec 19, 2025
 *      Author: Admin
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

void BNO055_Init(void);
void BNO055_ReadEuler_raw(int16_t *x, int16_t *y, int16_t *z);
uint8_t BNO055_ReadReg(uint8_t reg);
uint8_t BNO055_ReadCalibStatus(void);

#endif /* INC_BNO055_H_ */
