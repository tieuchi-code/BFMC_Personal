# Encoder lấy dữ liệu tốc độ xe
Sử dụng IC cảm biến từ trường AS6500

Dùng chung bus I2C với IMU (BNO055)
## Define cho AS5600
```c
/* ====== AS5600 I2C ====== */
#define AS5600_I2C_ADDR     0x36   // 7-bit
#define AS5600_STATUS       0x0B
#define AS5600_ANGLE_MSB    0x0E
#define AS5600_ANGLE_LSB    0x0F
```
0x36 là 7-bit, code đã shift (dev << 1)

## Hàm đọc STATUS (debug nam châm)
```c
static uint8_t AS5600_ReadStatus(void)
{
    uint8_t v = 0;
    I2C1_ReadMulti(AS5600_I2C_ADDR, AS5600_STATUS, &v, 1);
    return v;
}
```
**Ý nghĩa bit:**
- bit5 (MD) = 1 → có nam châm
- bit4 (ML) = 1 → từ yếu
- bit3 (MH) = 1 → từ mạnh

## Hàm đọc RAW ANGLE (12 bit)
```c
static uint16_t AS5600_ReadAngleRaw(void)
{
    uint8_t buf[2];
    I2C1_ReadMulti(AS5600_I2C_ADDR, AS5600_ANGLE_MSB, buf, 2);

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    raw &= 0x0FFF;   // lấy đúng 12 bit

    return raw;
}
```

## Hàm chuyển đổi về góc
```c
static float AS5600_ReadAngleDeg(void)
{
    uint16_t raw = AS5600_ReadAngleRaw();
    return (raw * 360.0f) / 4096.0f;
}
```
