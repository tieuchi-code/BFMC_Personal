# Các hàm UART trên STM32F411


### Hàm khởi tạo
PA9/PA10: 
    CHI_UART_init(0);
PB6/PB7: 
    CHI_UART_init(1);
```C
#include <stdint.h>
#include <math.h>
#include <string.h>

#define RCC_BASE_ADDR        0x40023800
#define GPIOA_BASE_ADDR      0x40020000
#define GPIOB_BASE_ADDR      0x40020400
#define I2C1_BASE_ADDR       0x40005400
#define USART1_BASE_ADDR	 0x40011000

/* ====== UART1: chọn PA9/PA10 (usePB=0) hoặc PB6/PB7 (usePB=1) ====== */
void CHI_UART_init(uint8_t usePB)
{
    uint32_t* RCC_AHB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x30);
    uint32_t* RCC_APB2ENR = (uint32_t*)(RCC_BASE_ADDR + 0x44);

    if (usePB) {
        *RCC_AHB1ENR |= (1 << 1); // GPIOB
    } else {
        *RCC_AHB1ENR |= (1 << 0); // GPIOA
    }
    *RCC_APB2ENR |= (1 << 4); // USART1

    if (usePB == 0) {
        /* ---- PA9 (TX), PA10 (RX) AF7 ---- */
        uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
        uint32_t* GPIOA_AFRH  = (uint32_t*)(GPIOA_BASE_ADDR + 0x24);

        // PA9 AF
        *GPIOA_MODER &= ~(0b11 << (9 * 2));
        *GPIOA_MODER |=  (0b10 << (9 * 2));
        *GPIOA_AFRH  &= ~(0xF << ((9-8) * 4));
        *GPIOA_AFRH  |=  (0x7 << ((9-8) * 4));

        // PA10 AF
        *GPIOA_MODER &= ~(0b11 << (10 * 2));
        *GPIOA_MODER |=  (0b10 << (10 * 2));
        *GPIOA_AFRH  &= ~(0xF << ((10-8) * 4));
        *GPIOA_AFRH  |=  (0x7 << ((10-8) * 4));
    } else {
        /* ---- PB6 (TX), PB7 (RX) AF7 ---- */
        uint32_t* GPIOB_MODER = (uint32_t*)(GPIOB_BASE_ADDR + 0x00);
        uint32_t* GPIOB_AFRL  = (uint32_t*)(GPIOB_BASE_ADDR + 0x20);

        // PB6 AF
        *GPIOB_MODER &= ~(0b11 << (6 * 2));
        *GPIOB_MODER |=  (0b10 << (6 * 2));
        *GPIOB_AFRL  &= ~(0xF << (6 * 4));
        *GPIOB_AFRL  |=  (0x7 << (6 * 4));

        // PB7 AF
        *GPIOB_MODER &= ~(0b11 << (7 * 2));
        *GPIOB_MODER |=  (0b10 << (7 * 2));
        *GPIOB_AFRL  &= ~(0xF << (7 * 4));
        *GPIOB_AFRL  |=  (0x7 << (7 * 4));
    }

    uint32_t* USART1_BRR = (uint32_t*)(USART1_BASE_ADDR + 0x08);
    *USART1_BRR = (104 << 4) | (3 << 0); // 9600 @16MHz

    uint32_t* USART1_CR1 = (uint32_t*)(USART1_BASE_ADDR + 0x0C);
    *USART1_CR1 &= ~(1 << 13);
    *USART1_CR1 &= ~(1 << 15); // oversampling 16
    *USART1_CR1 &= ~(1 << 12); // 8-bit
    *USART1_CR1 &= ~(1 << 10); // no parity
    *USART1_CR1 |= (1 << 2) | (1 << 3) | (1 << 13); // RE, TE, UE
}
```
### Hàm truyền dữ liệu
```C
void CHI_UART_Transmit(uint8_t data)
{
    uint32_t* USART_DR = (uint32_t*)(USART1_BASE_ADDR + 0x04);
    uint32_t* USART_SR = (uint32_t*)(USART1_BASE_ADDR + 0x00);
    while(((*USART_SR >> 7) & 1) == 0); // đợi TXE = 1 (data register empty)
    *USART_DR = data;
    while(((*USART_SR >> 6) & 1) == 0); // đợi TC = 1 (transmission complete)
}
```
### Hàm truyền số nguyên (dạng ký tự)
*Muốn truyền số phải include hàm transmit*
```C
void CHI_UART_send_number(int num)
{
    char buf[12];
    int i = 0;
    if (num == 0) {
        CHI_UART_Transmit('0');
        return;
    }
    if (num < 0) {
        CHI_UART_Transmit('-');
        num = -num;
    }
    while (num > 0) {
        buf[i++] = (num % 10) + '0';
        num /= 10;
    }
    while (i--) CHI_UART_Transmit(buf[i]);
}
```

### Hàm in log
```C
void CHI_UART_print_log(char *m)
{
    for (int i = 0; m[i]; i++) {
        CHI_UART_Transmit((uint8_t)m[i]);
    }
}
```


### Hàm truyền số thực
*Phải inlcude hàm transmit và hàm truyền số nguyên*
```C
void CHI_UART_send_float(float v)
{
    if (v < 0) {
        CHI_UART_Transmit('-');
        v = -v;
    }
    int ip = (int)v;
    int fp = (int)((v - ip) * 100); // 2 chữ số sau dấu chấm
    CHI_UART_send_number(ip);
    CHI_UART_Transmit('.');
    if (fp < 10) {
        CHI_UART_Transmit('0');
    }
    CHI_UART_send_number(fp);
    CHI_UART_Transmit('\r');
    CHI_UART_Transmit('\n');
}
```