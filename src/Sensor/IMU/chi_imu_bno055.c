#include "main.h"
#include "chi_imu_bno055.h"
#include <math.h>
#include <string.h>

#define RCC_BASE_ADDR        0x40023800
#define GPIOA_BASE_ADDR      0x40020000
#define GPIOB_BASE_ADDR      0x40020400
#define GPIOD_BASE_ADDR 	 0x40020C00
#define I2C1_BASE_ADDR       0x40005400
#define USART2_BASE_ADDR	 0x40004400

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
#define PWR_MODE_NORMAL      0x00
#define PWR_MODE_LOW_POWER   0x01
#define PWR_MODE_SUSPEND     0x02

#define BNO055_CHIP_ID_VAL   0xA0

/* ====== I2C1 Master (PB6=SCL, PB9=SDA) ====== */
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

void I2C1_WriteReg(uint8_t dev, uint8_t reg, uint8_t data)
{
    volatile uint32_t* CR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x00);
    volatile uint32_t* SR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x14);
    volatile uint32_t* SR2 = (uint32_t*)(I2C1_BASE_ADDR + 0x18);
    volatile uint32_t* DR  = (uint32_t*)(I2C1_BASE_ADDR + 0x10);

    *CR1 |= (1<<8);               // START
    while(((*SR1)&1)==0);         // SB
    *DR = (dev << 1);             // W
    while(((*SR1>>1)&1)==0);      // ADDR
    (void)*SR2;

    while(((*SR1>>7)&1)==0);      // TxE
    *DR = reg;

    while(((*SR1>>7)&1)==0);      // TxE
    *DR = data;

    while(((*SR1>>2)&1)==0);      // BTF
    *CR1 |= (1<<9);               // STOP
    while(*CR1 & (1<<9));
}

void I2C1_ReadMulti(uint8_t dev, uint8_t reg_start, uint8_t* buf, uint8_t len)
{
    volatile uint32_t* CR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x00);
    volatile uint32_t* SR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x14);
    volatile uint32_t* SR2 = (uint32_t*)(I2C1_BASE_ADDR + 0x18);
    volatile uint32_t* DR  = (uint32_t*)(I2C1_BASE_ADDR + 0x10);

    // Write reg address
    *CR1 |= (1<<8);
    while(!(*SR1 & 1));
    *DR = (dev << 1);             // W
    while(!(*SR1 & (1<<1)));
    (void)*SR2;

    while(!(*SR1 & (1<<7)));
    *DR = reg_start;              // (BNO không cần auto-inc flag)

    while(!(*SR1 & (1<<7)));      // TxE done

    // Re-START for Read
    *CR1 |= (1<<8);
    while(!(*SR1 & 1));
    *DR = (dev << 1) | 1;         // R
    while(!(*SR1 & (1<<1)));
    (void)*SR2;

    for (uint8_t i = 0; i < len; i++)
    {
        if (i == len - 1)
        {
            *CR1 &= ~(1<<10);     // ACK=0 cho byte cuối
            *CR1 |= (1<<9);       // STOP
        }
        while(!(*SR1 & (1<<6)));  // RxNE
        buf[i] = *DR;
    }
    *CR1 |= (1<<10); // bật lại ACK cho lần giao tiếp kế
}

//BNO055 helpers
uint8_t BNO055_ReadReg(uint8_t reg)
{
    uint8_t v=0;
    I2C1_ReadMulti(BNO055_I2C_ADDR, reg, &v, 1);
    return v;
}

/* Khởi tạo: CONFIG -> PAGE0 -> PWR NORMAL -> NDOF */
void BNO055_Init(void)
{
    // Về CONFIG mode
    I2C1_WriteReg(BNO055_I2C_ADDR, BNO055_OPR_MODE, OPR_MODE_CONFIG);
    HAL_Delay(25);

    // Khóa Page 0 để chắc chắn đúng bank thanh ghi
    I2C1_WriteReg(BNO055_I2C_ADDR, BNO055_PAGE_ID, 0x00);

    // Chọn Power mode NORMAL
    I2C1_WriteReg(BNO055_I2C_ADDR, BNO055_PWR_MODE, PWR_MODE_NORMAL);
    HAL_Delay(10);

    // Sang NDOF (Fusion 9DOF)
    I2C1_WriteReg(BNO055_I2C_ADDR, BNO055_OPR_MODE, OPR_MODE_NDOF);
    HAL_Delay(20);
}

/* Đọc Euler (Heading/Yaw, Roll, Pitch) – raw 1/16 độ */
void BNO055_ReadEuler_raw(int16_t* eul_x, int16_t* eul_y, int16_t* eul_z)
{
    uint8_t buf[6];
    I2C1_ReadMulti(BNO055_I2C_ADDR, BNO055_EUL_X_LSB, buf, 6);
    // Little-endian
    *eul_x = (int16_t)((buf[1] << 8) | buf[0]); // Heading/Yaw
    *eul_y = (int16_t)((buf[3] << 8) | buf[2]); // Roll
    *eul_z = (int16_t)((buf[5] << 8) | buf[4]); // Pitch
}

/* Đọc trạng thái hiệu chuẩn (SYS/ACC/GYR/MAG) */
uint8_t BNO055_ReadCalibStatus(void)
{
    return BNO055_ReadReg(BNO055_CALIB_STAT);
}
