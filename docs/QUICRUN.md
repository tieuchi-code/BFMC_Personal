# QUICRUN là động cơ kéo - động lực chính cho xe di chuyển 
Động cơ hoạt động bằng xung PWM, chu kì cố định 20ms (50Hz)
Xung dừng là 1.5ms
Xung đi tới là >1.5ms
Đi lùi là <1.5ms
Dao động từ 0.5ms -> 2.5ms  càng chênh lệch xung dừng càng nhanh

## Cấu hình PWM cho servo
Ví dụ trên chip STM32F411
```C
void PWM_init(void)
{
    // ==== 1. Enable clock cho GPIOA và TIM1 ====
    volatile uint32_t* RCC_AHB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x30);
    volatile uint32_t* RCC_APB2ENR = (uint32_t*)(RCC_BASE_ADDR + 0x44);
    *RCC_AHB1ENR |= (1 << 0);  // GPIOAEN
    *RCC_APB2ENR |= (1 << 0);  // TIM1EN

    // ==== 2. PA8 -> Alternate Function AF1 ====
    volatile uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
    volatile uint32_t* GPIOA_AFRH  = (uint32_t*)(GPIOA_BASE_ADDR + 0x24);

    *GPIOA_MODER &= ~(0b11 << (8 * 2));  // clear mode PA8
    *GPIOA_MODER |=  (0b10 << (8 * 2));  // set AF mode
    *GPIOA_AFRH  &= ~(0xF << ((8 - 8) * 4));
    *GPIOA_AFRH  |=  (0x1 << ((8 - 8) * 4)); // AF1 (TIM1_CH1)

    // ==== 3. Cấu hình Timer1 ====
    volatile uint32_t* TIM1_PSC  = (uint32_t*)(TIM1_BASE_ADDR + 0x28);
    volatile uint32_t* TIM1_ARR  = (uint32_t*)(TIM1_BASE_ADDR + 0x2C);
    volatile uint32_t* TIM1_CCR1 = (uint32_t*)(TIM1_BASE_ADDR + 0x34);
    volatile uint32_t* TIM1_CCMR1 = (uint32_t*)(TIM1_BASE_ADDR + 0x18);
    volatile uint32_t* TIM1_CCER  = (uint32_t*)(TIM1_BASE_ADDR + 0x20);
    volatile uint32_t* TIM1_BDTR  = (uint32_t*)(TIM1_BASE_ADDR + 0x44);
    volatile uint32_t* TIM1_CR1   = (uint32_t*)(TIM1_BASE_ADDR + 0x00);
    volatile uint32_t* TIM1_EGR   = (uint32_t*)(TIM1_BASE_ADDR + 0x14);

    *TIM1_PSC = 16-1;       // chia 16MHz/16 = 1MHz tick
    *TIM1_ARR = 20000-200;      // chu kỳ 1000 tick => PWM tần số = 1kHz
    *TIM1_CCR1 = 500-1;     // duty 50%

    // ==== 4. Chọn PWM mode 1, bật preload ====
    *TIM1_CCMR1 &= ~(0xFF << 0);
    *TIM1_CCMR1 |= (6 << 4) | (1 << 3);  // OC1M=110, OC1PE=1

    // ==== 5. Bật output channel 1 ====
    *TIM1_CCER |= (1 << 0);   // CC1E=1, active high

    // ==== 6. Bật Main Output Enable (timer advanced) ====
    *TIM1_BDTR |= (1 << 15);  // MOE=1

    // ==== 7. Kích hoạt update để nạp giá trị ====
    *TIM1_EGR = 1;            // UG=1

    // ==== 8. Bật timer ====
    *TIM1_CR1 |= (1 << 0);    // CEN=1
}
```
## Hàm điều khiển tốc độ
Truyền vào giá trị từ 500 -> 2500
```C
void PWM_servo_speed(int width)
{
    volatile uint32_t* TIM1_CCR1 = (uint32_t*)(TIM1_BASE_ADDR + 0x34);
    *TIM1_CCR1 = width ;
}
```


