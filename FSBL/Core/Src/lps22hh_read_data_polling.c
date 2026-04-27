/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  MEMS Software Solution Team
 * @brief   This file shows how to get data from sensor .
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
 * - STEVAL_MKI109V3 + STEVAL-MKI192V1
 * - NUCLEO_F401RE + X_NUCLEO_IKS01A3
 * - DISCOVERY_SPC584B + STEVAL-MKI192V1
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
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
#include "lps22hh_reg.h"
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
#define    BOOT_TIME        5 //ms

#define TX_BUF_DIM          1000

/* Private variables ---------------------------------------------------------*/
static uint32_t data_raw_pressure;
static int16_t data_raw_temperature;
static float_t pressure_hPa;
static float_t temperature_degC;
static uint8_t whoamI, rst;


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

static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/

void lps22hh_read_data_polling(void)
{
  stmdev_ctx_t dev_ctx;
  lps22hh_status_t status;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  platform_init();
  platform_delay(BOOT_TIME);

  whoamI = 0;
  lps22hh_device_id_get(&dev_ctx, &whoamI);
  //printf("LPS22HH whoamI=0x%02X\r\n", whoamI);  // should print 0xB3

  if (whoamI != LPS22HH_ID) {
    printf("LPS22HH not found\r\n");
    return;  // exit instead of while(1)
  }

  /* Restore default configuration */
  lps22hh_reset_set(&dev_ctx, PROPERTY_ENABLE);
  int reset_timeout = 0;
  do {
    lps22hh_reset_get(&dev_ctx, &rst);
    platform_delay(10);
    reset_timeout++;
  } while (rst && reset_timeout < 50);

  /* Enable Block Data Update */
  lps22hh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lps22hh_data_rate_set(&dev_ctx, LPS22HH_10_Hz_LOW_NOISE);

  /* Wait for first measurement (10Hz = 100ms per sample) */
  platform_delay(200);

  /* Wait until both pressure and temperature are ready with timeout */
  int timeout = 0;
  do {
    lps22hh_read_reg(&dev_ctx, LPS22HH_STATUS, (uint8_t *)&status, 1);
    platform_delay(50);
    timeout++;
  } while ((!status.p_da || !status.t_da) && timeout < 20);  // wait up to 1 second

  /* Read pressure */
  memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
  lps22hh_pressure_raw_get(&dev_ctx, &data_raw_pressure);
  int32_t pressure_centi_hpa = ((int64_t)data_raw_pressure * 100) / 1048576;
  int32_t pressure_abs_centi_hpa = (pressure_centi_hpa < 0) ? -pressure_centi_hpa : pressure_centi_hpa;
    printf("pressure [hPa]:%ld.%02ld\r\n",
      (long)(pressure_centi_hpa / 100),
      (long)(pressure_abs_centi_hpa % 100));

  /* Read temperature */
  memset(&data_raw_temperature, 0x00, sizeof(int16_t));
  lps22hh_temperature_raw_get(&dev_ctx, &data_raw_temperature);
  int32_t temp_centi = data_raw_temperature;
  int32_t temp_abs_centi = (temp_centi < 0) ? -temp_centi : temp_centi;
  printf("temperature [degC]:%ld.%02ld\r\n",
         (long)(temp_centi / 100),
         (long)(temp_abs_centi % 100));
}
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
    HAL_I2C_Mem_Write(handle, LPS22HH_I2C_ADD_H, reg,  // was LIS2DW12_I2C_ADD_H
                      I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000);
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    HAL_I2C_Mem_Read(handle, LPS22HH_I2C_ADD_H, reg,   // was LIS2DW12_I2C_ADD_H
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    return 0;
}

static void platform_delay(uint32_t ms)
{
    HAL_Delay(ms);
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
  HAL_Delay(1000);
#endif
}
