#include "main.h"
#include <math.h>
#include <string.h>

#define USART1_BASE_ADDR   0x40011000UL
#define RCC_BASE_ADDR        0x40023800
#define GPIOA_BASE_ADDR      0x40020000
#define GPIOB_BASE_ADDR      0x40020400
#define GPIOD_BASE_ADDR      0x40020C00
#define I2C1_BASE_ADDR       0x40005400
/* ====== BNO055 I2C ====== */
#define BNO055_I2C_ADDR      0x28   // 7-bit
#define BNO055_CHIP_ID       0x00
#define BNO055_PAGE_ID       0x07
#define BNO055_EUL_X_LSB     0x1A   // Heading (Yaw)
#define BNO055_EUL_X_MSB     0x1B
#define BNO055_EUL_Y_LSB     0x1C   // Roll
#define BNO055_EUL_Y_MSB     0x1D
#define BNO055_EUL_Z_LSB     0x1E   // Pitch
#define BNO055_EUL_Z_MSB     0x1F
#define BNO055_CALIB_STAT    0x35
#define BNO055_UNIT_SEL      0x3B
#define BNO055_OPR_MODE      0x3D
#define BNO055_PWR_MODE      0x3E
#define BNO055_SYS_TRIGGER   0x3F

/* OPR_MODE values */
#define OPR_MODE_CONFIG      0x00
#define OPR_MODE_NDOF        0x0C   // Fusion 9DOF

/* PWR_MODE values */
#define BNO055_PWR_NORMAL      0x00
#define PWR_MODE_LOW_POWER   0x01
#define PWR_MODE_SUSPEND     0x02

#define BNO055_CHIP_ID_VAL   0xA0

void UART_init(void)
{
    /* ===== Enable clocks ===== */
    uint32_t* RCC_AHB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x30);
    uint32_t* RCC_APB2ENR = (uint32_t*)(RCC_BASE_ADDR + 0x44);

    *RCC_AHB1ENR |= (1 << 0);    // GPIOA clock
    *RCC_APB2ENR |= (1 << 4);    // USART1 clock

    /* ===== GPIO PA9, PA10 alternate function ===== */
    uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
    uint32_t* GPIOA_AFRH  = (uint32_t*)(GPIOA_BASE_ADDR + 0x24);

    /* PA9 -> USART1_TX (AF7) */
    *GPIOA_MODER &= ~(0b11 << (9 * 2));
    *GPIOA_MODER |=  (0b10 << (9 * 2));
    *GPIOA_AFRH  &= ~(0xF << ((9 - 8) * 4));
    *GPIOA_AFRH  |=  (0x7 << ((9 - 8) * 4));

    /* PA10 -> USART1_RX (AF7) */
    *GPIOA_MODER &= ~(0b11 << (10 * 2));
    *GPIOA_MODER |=  (0b10 << (10 * 2));
    *GPIOA_AFRH  &= ~(0xF << ((10 - 8) * 4));
    *GPIOA_AFRH  |=  (0x7 << ((10 - 8) * 4));

    /* ===== USART1 configuration ===== */
    uint32_t* USART1_BRR = (uint32_t*)(USART1_BASE_ADDR + 0x08);
    uint32_t* USART1_CR1 = (uint32_t*)(USART1_BASE_ADDR + 0x0C);

    /* Baudrate = 9600, PCLK2 = 16 MHz
       USARTDIV = 16MHz / (16 * 9600) = 104.166 */
    *USART1_BRR = (104 << 4) | (3 << 0);

    *USART1_CR1 = 0;
    *USART1_CR1 &= ~(1 << 15);   // Oversampling 16
    *USART1_CR1 &= ~(1 << 12);   // 8-bit data
    *USART1_CR1 &= ~(1 << 10);   // No parity
    *USART1_CR1 |=  (1 << 2);    // RE
    *USART1_CR1 |=  (1 << 3);    // TE
    *USART1_CR1 |=  (1 << 13);   // UE
}


void UART_Transmit(uint8_t data)
{
    uint32_t* USART_DR= (uint32_t*)(USART1_BASE_ADDR + 0x04);
    uint32_t* USART_SR= (uint32_t*)(USART1_BASE_ADDR + 0x00);
    while(((*USART_SR >> 7) & 1) == 0); // TXE
    *USART_DR = data;
    while(((*USART_SR >> 6) & 1) == 0); // TC
}

void UART_send_number(int num)
{
    char buf[12]; int i = 0;
    if (num == 0) { UART_Transmit('0'); return; }
    if (num < 0) { UART_Transmit('-'); num = -num; }
    while (num > 0) { buf[i++] = (num%10)+'0'; num/=10; }
    while (i--) UART_Transmit(buf[i]);
}
void UART_print_log(char *m)
{
    for (int i = 0; m[i]; i++) UART_Transmit((uint8_t)m[i]);
}
void UART_send_float(float v)
{
    if (v < 0) { UART_Transmit('-'); v = -v; }
    int ip = (int)v;
    int fp = (int)((v - ip) * 100);
    UART_send_number(ip); UART_Transmit('.');
    if (fp < 10) UART_Transmit('0');
    UART_send_number(fp);
    UART_Transmit('\r'); UART_Transmit('\n');
}

void I2C1_Master_Init()
{
    volatile uint32_t* RCC_AHB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x30);
    volatile uint32_t* RCC_APB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x40);
    *RCC_AHB1ENR |= (1 << 1); // GPIOB
    *RCC_APB1ENR |= (1 << 21); // I2C1

    volatile uint32_t* MODER   = (uint32_t*)(GPIOB_BASE_ADDR + 0x00);
    volatile uint32_t* OTYPER  = (uint32_t*)(GPIOB_BASE_ADDR + 0x04);
    volatile uint32_t* OSPEEDR = (uint32_t*)(GPIOB_BASE_ADDR + 0x08);
    volatile uint32_t* PUPDR   = (uint32_t*)(GPIOB_BASE_ADDR + 0x0C);
    volatile uint32_t* AFRL    = (uint32_t*)(GPIOB_BASE_ADDR + 0x20);
    volatile uint32_t* AFRH    = (uint32_t*)(GPIOB_BASE_ADDR + 0x24);

    *MODER &= ~((0b11<<(6*2)) | (0b11<<(9*2)));
    *MODER |=  ((0b10<<(6*2)) | (0b10<<(9*2)));  // AF
    *OTYPER |= (1<<6) | (1<<9);                  // OD
    *OSPEEDR |= (0b11<<(6*2)) | (0b11<<(9*2));   // High
    *PUPDR &= ~((0b11<<(6*2)) | (0b11<<(9*2)));
    *PUPDR |=  ((0b01<<(6*2)) | (0b01<<(9*2)));  // Pull-up
    *AFRL &= ~(0xF<<(6*4));  *AFRL |= (0x4<<(6*4));
    *AFRH &= ~(0xF<<((9-8)*4)); *AFRH |= (0x4<<((9-8)*4));

    volatile uint32_t* CR1   = (uint32_t*)(I2C1_BASE_ADDR + 0x00);
    volatile uint32_t* CR2   = (uint32_t*)(I2C1_BASE_ADDR + 0x04);
    volatile uint32_t* CCR   = (uint32_t*)(I2C1_BASE_ADDR + 0x1C);
    volatile uint32_t* TRISE = (uint32_t*)(I2C1_BASE_ADDR + 0x20);

    *CR1 &= ~(1<<0);   // PE=0
    *CR2 = 16;         // PCLK1=16MHz
    *CCR = 80;         // 100kHz
    *TRISE = 17;       // 1000ns @16MHz
    *CR1 |= (1<<10);   // ACK
    *CR1 |= (1<<0);    // PE=1
}
void I2C_master_transmit(uint8_t address, uint8_t data)
{
    volatile uint32_t* CR1 = (uint32_t*)(I2C1_BASE + 0x00);
    volatile uint32_t* SR1 = (uint32_t*)(I2C1_BASE + 0x14);
    volatile uint32_t* SR2 = (uint32_t*)(I2C1_BASE + 0x18);
    volatile uint32_t* DR  = (uint32_t*)(I2C1_BASE + 0x10);

    // 1. Gửi START
    *CR1 |= (1 << 8);
    while (!(*SR1 & (1 << 0))); // Wait SB = 1 (sent start)

    // 2. Gửi địa chỉ + Write
    *DR = (address << 1);
    while (!(*SR1 & (1 << 1)));	// ADDR = 1
    (void)*SR2;					// Clear cờ addr

    // 3. Gửi dữ liệu
    while (!(*SR1 & (1 << 7))); // TxE = 1 (DR empty)
    *DR = data;

    // 4. Đợi truyền xong
    while (!(*SR1 & (1 << 2))); // BTF = 1

    // 5. Gửi STOP
    *CR1 |= (1 << 9);
}



int main(void)
{
    HAL_Init();
    UART_init();
    I2C1_Master_Init();
    I2C_master_transmit(0x28,0x40);
    while (1)
    {

    }
}


