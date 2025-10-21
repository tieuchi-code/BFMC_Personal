#include <stdint.h>
#include "chi_imu_bno055.h"
#include <math.h>
#include <string.h>

#define RCC_BASE_ADDR        0x40023800
#define GPIOA_BASE_ADDR      0x40020000
#define GPIOB_BASE_ADDR      0x40020400
#define GPIOD_BASE_ADDR 	 0x40020C00
#define I2C1_BASE_ADDR       0x40005400
#define USART2_BASE_ADDR	 0x40004400

/* ====== UART2 (PA2/PA3) ====== */
void CHI_UART_init()
{
    uint32_t* RCC_AHB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x30);
    *RCC_AHB1ENR |= (1 << 0); // GPIOA

    uint32_t* RCC_APB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x40);
    *RCC_APB1ENR |= (1 << 17); // USART2

    uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
    uint32_t* GPIOA_AFRL  = (uint32_t*)(GPIOA_BASE_ADDR + 0x20);

    // PA2 AF7
    *GPIOA_MODER &= ~(0b11 << (2 * 2));
    *GPIOA_MODER |=  (0b10 << (2 * 2));
    *GPIOA_AFRL  &= ~(0xF << (2 * 4));
    *GPIOA_AFRL  |=  (0x7 << (2 * 4));

    // PA3 AF7
    *GPIOA_MODER &= ~(0b11 << (3 * 2));
    *GPIOA_MODER |=  (0b10 << (3 * 2));
    *GPIOA_AFRL  &= ~(0xF << (3 * 4));
    *GPIOA_AFRL  |=  (0x7 << (3 * 4));

    uint32_t* USART2_BRR = (uint32_t*)(USART2_BASE_ADDR + 0x08);
    *USART2_BRR = (104 << 4) | (3 << 0); // 9600 @16MHz

    uint32_t* USART2_CR1 = (uint32_t*)(USART2_BASE_ADDR + 0x0C);
    *USART2_CR1 &= ~(1 << 13);
    *USART2_CR1 &= ~(1 << 15); // oversampling 16
    *USART2_CR1 &= ~(1 << 12); // 8-bit
    *USART2_CR1 &= ~(1 << 10); // no parity
    *USART2_CR1 |= (1 << 2) | (1 << 3) | (1 << 13); // RE, TE, UE
}

void CHI_UART_Transmit(uint8_t data)
{
    uint32_t* USART_DR= (uint32_t*)(USART2_BASE_ADDR + 0x04);
    uint32_t* USART_SR= (uint32_t*)(USART2_BASE_ADDR + 0x00);
    while(((*USART_SR >> 7) & 1) == 0); // TXE
    *USART_DR = data;
    while(((*USART_SR >> 6) & 1) == 0); // TC
}

void CHI_UART_send_number(int num)
{
    char buf[12]; int i = 0;
    if (num == 0) { CHI_UART_Transmit('0'); return; }
    if (num < 0) { CHI_UART_Transmit('-'); num = -num; }
    while (num > 0) { buf[i++] = (num%10)+'0'; num/=10; }
    while (i--) CHI_UART_Transmit(buf[i]);
}
void CHI_UART_print_log(char *m)
{
    for (int i = 0; m[i]; i++) CHI_UART_Transmit((uint8_t)m[i]);
}
void CHI_UART_send_float(float v)
{
    if (v < 0) { CHI_UART_Transmit('-'); v = -v; }
    int ip = (int)v;
    int fp = (int)((v - ip) * 100);
    CHI_UART_send_number(ip); CHI_UART_Transmit('.');
    if (fp < 10) CHI_UART_Transmit('0');
    CHI_UART_send_number(fp);
    CHI_UART_Transmit('\r'); CHI_UART_Transmit('\n');
}
