/*
 * uart.h
 *
 *  Created on: Dec 19, 2025
 *      Author: Admin
 */

#ifndef INC_UART_H_
#define INC_UART_H_



void CHI_UART_Transmit(uint8_t data);
void CHI_UART_init(void);
void CHI_UART_send_number(int num);
void CHI_UART_print_log(char *m);
void CHI_UART_send_float(float v);

#endif /* INC_UART_H_ */
