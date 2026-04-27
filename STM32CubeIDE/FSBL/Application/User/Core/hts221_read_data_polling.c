/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "hts221_reg.h"
#include "stm32n6xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
#define SENSOR_BUS hi2c1

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_humidity;
static int16_t data_raw_temperature;
static float_t humidity_perc;
static float_t temperature_degC;
static uint8_t whoamI;

/* Private functions ---------------------------------------------------------*/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main function -------------------------------------------------------------*/
void hts221_read_data_polling(void)
{
  platform_init();

  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg  = platform_read;
  dev_ctx.mdelay    = platform_delay;
  dev_ctx.handle    = &SENSOR_BUS;

  /* Check device ID */
  whoamI = 0;
  hts221_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != HTS221_ID) {
    printf("HTS221 not found\r\n");
    return;
  }

  /* Calibration variables */
  uint8_t h0_rh_x2, h1_rh_x2;
  uint8_t cal_buf[2];
  int16_t h0_t0_out, h1_t0_out;

  platform_read(&hi2c1, 0x30, &h0_rh_x2, 1);
  platform_read(&hi2c1, 0x31, &h1_rh_x2, 1);
  platform_read(&hi2c1, 0x36, cal_buf, 2);
  h0_t0_out = (int16_t)((cal_buf[1] << 8) | cal_buf[0]);
  platform_read(&hi2c1, 0x3A, cal_buf, 2);
  h1_t0_out = (int16_t)((cal_buf[1] << 8) | cal_buf[0]);

  uint8_t t0_degc_x8, t1_degc_x8;
  int16_t t0_out, t1_out;

  platform_read(&hi2c1, 0x32, &t0_degc_x8, 1);
  platform_read(&hi2c1, 0x33, &t1_degc_x8, 1);
  platform_read(&hi2c1, 0x3C, cal_buf, 2);
  t0_out = (int16_t)((cal_buf[1] << 8) | cal_buf[0]);
  platform_read(&hi2c1, 0x3E, cal_buf, 2);
  t1_out = (int16_t)((cal_buf[1] << 8) | cal_buf[0]);

  /* Enable sensor */
  hts221_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  hts221_data_rate_set(&dev_ctx, HTS221_ODR_1Hz);
  hts221_power_on_set(&dev_ctx, PROPERTY_ENABLE);

  HAL_Delay(1500);

  /* Wait data ready */
  hts221_status_reg_t status;
  do {
    hts221_status_get(&dev_ctx, &status);
    HAL_Delay(100);
  } while (!status.h_da || !status.t_da);

  /* Read raw data */
  hts221_humidity_raw_get(&dev_ctx, &data_raw_humidity);
  hts221_temperature_raw_get(&dev_ctx, &data_raw_temperature);

  /* ===== HUMIDITY ===== */
  int32_t h0 = (h0_rh_x2 / 2) * 100;
  int32_t h1 = (h1_rh_x2 / 2) * 100;

  int32_t humidity_centi = h0 +
    ((h1 - h0) * (data_raw_humidity - h0_t0_out)) /
    (h1_t0_out - h0_t0_out);

  if (humidity_centi < 0) humidity_centi = 0;
  if (humidity_centi > 10000) humidity_centi = 10000;

  /* ===== TEMPERATURE ===== */
  int32_t t0 = (t0_degc_x8 / 8) * 100;
  int32_t t1 = (t1_degc_x8 / 8) * 100;

  int32_t temp_centi = t0 +
    ((t1 - t0) * (data_raw_temperature - t0_out)) /
    (t1_out - t0_out);

  /* ✅ FIX IMPORTANT */
  humidity_perc   = (float)humidity_centi / 100.0f;
  temperature_degC = (float)temp_centi / 100.0f;

  /* Print */
  printf("Humidity [%%]:%.2f\r\n", humidity_perc);
  printf("Temperature [degC]:%.2f\r\n", temperature_degC);
}

/* Platform I2C write */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  reg |= 0x80;
  if (HAL_I2C_Mem_Write(handle, HTS221_I2C_ADDRESS, reg,
                        I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000) != HAL_OK)
    return -1;
  return 0;
}

/* Platform I2C read */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  reg |= 0x80;
  if (HAL_I2C_Mem_Read(handle, HTS221_I2C_ADDRESS, reg,
                       I2C_MEMADD_SIZE_8BIT, bufp, len, 1000) != HAL_OK)
    return -1;
  return 0;
}

static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

static void platform_init(void)
{
}

/* ===== GETTERS ===== */
float hts221_get_temperature(void)
{
  return temperature_degC;
}

float hts221_get_humidity(void)
{
  return humidity_perc;
}
