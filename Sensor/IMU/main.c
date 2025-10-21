#include "main.h"
#include "chi_imu_bno055.h"

#define BNO055_CHIP_ID_VAL   0xA0
#define BNO055_CHIP_ID       0x00


int main(void)
{
    HAL_Init();
    CHI_UART_init();
    I2C1_Master_Init();

    CHI_UART_print_log("BNO055 scan...\r\n");
    uint8_t id = BNO055_ReadReg(BNO055_CHIP_ID);
    CHI_UART_print_log("CHIP_ID="); CHI_UART_send_number(id); CHI_UART_print_log("\r\n");
    if (id != BNO055_CHIP_ID_VAL) {
    	CHI_UART_print_log("BNO055 not found (addr/PS/pull-up?)\r\n");
        while(1);
    }

    CHI_UART_print_log("BNO055 init...\r\n");
    BNO055_Init();

    HAL_Delay(50);

    while (1)
    {
        int16_t ex, ey, ez;     // raw (1/16 deg)
        float   yaw, roll, pitch;

        BNO055_ReadEuler_raw(&ex, &ey, &ez);

        yaw   =  ex / 16.0f;    // Heading/Yaw
        roll  =  ey / 16.0f;    // Roll
        pitch =  ez / 16.0f;    // Pitch

        CHI_UART_print_log("Yaw: ");   CHI_UART_send_float(yaw);
        CHI_UART_print_log("Pitch: "); CHI_UART_send_float(pitch);
        CHI_UART_print_log("Roll: ");  CHI_UART_send_float(roll);

        HAL_Delay(100);
    }
}
