#include <string.h>
#include <stdio.h>
#include "lps22hh_reg.h"
#include "stm32n6xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
#define SENSOR_BUS hi2c1

#define BOOT_TIME 5 // ms

/* Private variables */
static uint32_t data_raw_pressure;
static int16_t data_raw_temperature;
static float_t pressure_hPa;
static float_t temperature_degC;
static uint8_t whoamI, rst;

/* Private functions */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* ================= MAIN FUNCTION ================= */
void lps22hh_read_data_polling(void)
{
  stmdev_ctx_t dev_ctx;
  lps22hh_status_t status;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg  = platform_read;
  dev_ctx.mdelay    = platform_delay;
  dev_ctx.handle    = &SENSOR_BUS;

  platform_init();
  platform_delay(BOOT_TIME);

  /* Check sensor ID */
  whoamI = 0;
  lps22hh_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LPS22HH_ID)
  {
    printf("LPS22HH not found\r\n");
    return;
  }

  /* Reset sensor */
  lps22hh_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lps22hh_reset_get(&dev_ctx, &rst);
    platform_delay(10);
  } while (rst);

  /* Configure sensor */
  lps22hh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  lps22hh_data_rate_set(&dev_ctx, LPS22HH_10_Hz_LOW_NOISE);

  platform_delay(200);

  /* Wait data ready */
  do {
    lps22hh_read_reg(&dev_ctx, LPS22HH_STATUS, (uint8_t *)&status, 1);
    platform_delay(50);
  } while (!status.p_da || !status.t_da);

  /* ===== PRESSURE ===== */
  lps22hh_pressure_raw_get(&dev_ctx, &data_raw_pressure);

  int32_t pressure_centi = ((int64_t)data_raw_pressure * 100) / 1048576;

  pressure_hPa = (float)pressure_centi / 100.0f;   // ✅ FIX IMPORTANT

  printf("pressure [hPa]:%.2f\r\n", pressure_hPa);

  /* ===== TEMPERATURE ===== */
  lps22hh_temperature_raw_get(&dev_ctx, &data_raw_temperature);

  int32_t temp_centi = data_raw_temperature;

  temperature_degC = (float)temp_centi / 100.0f;   // optionnel

  printf("temperature [degC]:%.2f\r\n", temperature_degC);
}

/* ================= I2C WRITE ================= */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  if (HAL_I2C_Mem_Write(handle, LPS22HH_I2C_ADD_H, reg,
                        I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000) != HAL_OK)
  {
    return -1;
  }
  return 0;
}

/* ================= I2C READ ================= */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  if (HAL_I2C_Mem_Read(handle, LPS22HH_I2C_ADD_H, reg,
                       I2C_MEMADD_SIZE_8BIT, bufp, len, 1000) != HAL_OK)
  {
    return -1;
  }
  return 0;
}

/* ================= DELAY ================= */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

/* ================= INIT ================= */
static void platform_init(void)
{
}

/* ================= GETTER ================= */
float lps22hh_get_pressure(void)
{
  return pressure_hPa;
}
