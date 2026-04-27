/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  MEMS Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI141V2
 * - NUCLEO_F401RE + STEVAL-MKI141V2
 * - DISCOVERY_SPC584B + STEVAL-MKI141V2
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(N/A)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */

//#define STEVAL_MKI109V3  /* little endian */
//#define NUCLEO_F401RE    /* little endian */
//#define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2
/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F401RE)
/* NUCLEO_F401RE: Define communication interface */
#define SENSOR_BUS hi2c1

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS I2CD1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "hts221_reg.h"
#include "stm32n6xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
#define SENSOR_BUS hi2c1

#if defined(NUCLEO_F401RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#elif defined(STEVAL_MKI109V3)
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"

#elif defined(SPC584B_DIS)
#include "components.h"
#endif

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_humidity;
static int16_t data_raw_temperature;
static float_t humidity_perc;
static float_t temperature_degC;
static uint8_t whoamI;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

/*
 *  Function used to apply coefficient
 */
typedef struct {
  float_t x0;
  float_t y0;
  float_t x1;
  float_t y1;
} lin_t;

float_t linear_interpolation(lin_t *lin, int16_t x)
{
  return ((lin->y1 - lin->y0) * x + ((lin->x1 * lin->y0) -
                                     (lin->x0 * lin->y1)))
         / (lin->x1 - lin->x0);
}
void hts221_read_data_polling(void)
{
  
  /* Initialize platform specific hardware */
  platform_init();
  
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  
  /* Check device ID */
  whoamI = 0;
  hts221_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != HTS221_ID) {
    printf("HTS221 not found\r\n");
    return;  /* exit instead of blocking */
  }

  /* Skip problematic calibration reads - just use default values */
  lin_t lin_hum = {0};
  lin_t lin_temp = {0};

  /* Read calibration data directly from registers with timeout */
  uint8_t h0_rh_x2, h1_rh_x2;
  uint8_t cal_buf[2];
  int16_t h0_t0_out, h1_t0_out;
  
  /* Read H0_rH_x2 (0x30) with timeout - stored as 2x value */
  int cal_timeout = 0;
  while (platform_read(&hi2c1, 0x30, &h0_rh_x2, 1) != 0 && cal_timeout < 5) {
    HAL_Delay(10);
    cal_timeout++;
  }
  
  /* Read H1_rH_x2 (0x31) with timeout - stored as 2x value */
  cal_timeout = 0;
  while (platform_read(&hi2c1, 0x31, &h1_rh_x2, 1) != 0 && cal_timeout < 5) {
    HAL_Delay(10);
    cal_timeout++;
  }
  
  /* Read H0_T0_OUT (0x36-0x37) with timeout */
  cal_timeout = 0;
  while (platform_read(&hi2c1, 0x36, cal_buf, 2) != 0 && cal_timeout < 5) {
    HAL_Delay(10);
    cal_timeout++;
  }
  h0_t0_out = (int16_t)((cal_buf[1] << 8) | cal_buf[0]);
  
  /* Read H1_T0_OUT (0x3A-0x3B) with timeout */
  cal_timeout = 0;
  while (platform_read(&hi2c1, 0x3A, cal_buf, 2) != 0 && cal_timeout < 5) {
    HAL_Delay(10);
    cal_timeout++;
  }
  h1_t0_out = (int16_t)((cal_buf[1] << 8) | cal_buf[0]);
  
  /* Read temperature calibration: T0_degC_x8 (0x32) and T1_degC_x8 (0x33) */
  uint8_t t0_degc_x8, t1_degc_x8;
  cal_timeout = 0;
  while (platform_read(&hi2c1, 0x32, &t0_degc_x8, 1) != 0 && cal_timeout < 5) {
    HAL_Delay(10);
    cal_timeout++;
  }
  
  cal_timeout = 0;
  while (platform_read(&hi2c1, 0x33, &t1_degc_x8, 1) != 0 && cal_timeout < 5) {
    HAL_Delay(10);
    cal_timeout++;
  }
  
  /* Read T0_OUT (0x3C-0x3D) with timeout */
  int16_t t0_out;
  cal_timeout = 0;
  while (platform_read(&hi2c1, 0x3C, cal_buf, 2) != 0 && cal_timeout < 5) {
    HAL_Delay(10);
    cal_timeout++;
  }
  t0_out = (int16_t)((cal_buf[1] << 8) | cal_buf[0]);
  
  /* Read T1_OUT (0x3E-0x3F) with timeout */
  int16_t t1_out;
  cal_timeout = 0;
  while (platform_read(&hi2c1, 0x3E, cal_buf, 2) != 0 && cal_timeout < 5) {
    HAL_Delay(10);
    cal_timeout++;
  }
  t1_out = (int16_t)((cal_buf[1] << 8) | cal_buf[0]);
  
  /* Enable Block Data Update */
  hts221_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  
  /* Set Output Data Rate */
  hts221_data_rate_set(&dev_ctx, HTS221_ODR_1Hz);
  
  /* Device power on */
  hts221_power_on_set(&dev_ctx, PROPERTY_ENABLE);

  /* Wait for first measurement to be ready */
  HAL_Delay(1500);

  /* Wait until data is available with timeout */
  hts221_status_reg_t status;
  int timeout = 0;
  do {
    hts221_status_get(&dev_ctx, &status);
    HAL_Delay(100);
    timeout++;
  } while ((!status.h_da || !status.t_da) && timeout < 10);  

  /* Read humidity data */
  memset(&data_raw_humidity, 0x00, sizeof(int16_t));
  hts221_humidity_raw_get(&dev_ctx, &data_raw_humidity);
  
  /* Apply linear interpolation with calibration: humidity = H0_rH + (H1_rH - H0_rH) * (raw - H0_OUT) / (H1_OUT - H0_OUT) */
  /* Note: H0_rH_x2 and H1_rH_x2 are stored as 2x values, so divide by 2 */
  /* Multiply by 100 to get centi-percent */
  int32_t h0_rh_centi = (h0_rh_x2 / 2) * 100;
  int32_t h1_rh_centi = (h1_rh_x2 / 2) * 100;
  int32_t humidity_centi = h0_rh_centi + 
                           (((h1_rh_centi - h0_rh_centi) * ((int32_t)data_raw_humidity - h0_t0_out)) / 
                            (h1_t0_out - h0_t0_out));
  
  /* Clamp to 0-10000 (0-100%) */
  if (humidity_centi < 0) humidity_centi = 0;
  if (humidity_centi > 10000) humidity_centi = 10000;

  /* Read temperature data */
  memset(&data_raw_temperature, 0x00, sizeof(int16_t));
  hts221_temperature_raw_get(&dev_ctx, &data_raw_temperature);
  
  /* Apply linear interpolation with calibration for temperature */
  /* T0_degC_x8 and T1_degC_x8 are stored as 8x values, so divide by 8 */
  /* Result in centi-degrees */
  int32_t t0_centi = (t0_degc_x8 / 8) * 100;
  int32_t t1_centi = (t1_degc_x8 / 8) * 100;
  int32_t temp_centi = t0_centi + 
                       (((t1_centi - t0_centi) * ((int32_t)data_raw_temperature - t0_out)) / 
                        (t1_out - t0_out));
  int32_t temp_abs_centi = (temp_centi < 0) ? -temp_centi : temp_centi;

  printf("Humidity [%%]:%ld.%02ld\r\n",
    (long)(humidity_centi / 100),
    (long)(humidity_centi % 100));
  printf("Temperature [degC]:%ld.%02ld\r\n",
    (long)(temp_centi / 100),
    (long)(temp_abs_centi % 100));

}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_StatusTypeDef status;

  /* Write multiple command */
  reg |= 0x80;
  status = HAL_I2C_Mem_Write(handle, HTS221_I2C_ADDRESS, reg,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

  if (status != HAL_OK) {
    printf("HTS221 I2C Write Error\r\n");
    /* Try to recover I2C bus */
    HAL_I2C_DeInit(handle);
    HAL_Delay(10);
    HAL_I2C_Init(handle);
    HAL_Delay(10);
    return -1;
  }
  
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_StatusTypeDef status;

  /* Read multiple command */
  reg |= 0x80;
  status = HAL_I2C_Mem_Read(handle, HTS221_I2C_ADDRESS, reg,
                            I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  if (status != HAL_OK) {
    /* Try to recover I2C bus */
    HAL_I2C_DeInit(handle);
    HAL_Delay(10);
    HAL_I2C_Init(handle);
    HAL_Delay(10);
    return -1;
  }
  
  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */


/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
#if defined(NUCLEO_F401RE) | defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#if defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  platform_delay(1000);
#endif
}
