/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  Sensors Software Solution Team
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
 * - STEVAL_MKI109V3 + STEVAL-MKI179V1
 * - NUCLEO_F401RE + X_NUCLEO_IKS01A3
 * - DISCOVERY_SPC584B + X_NUCLEO_IKS01A3
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
#include "lis2dw12_reg.h"

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
#define    BOOT_TIME            20 //ms

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static float_t acceleration_mg[3];
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

void lis2dw12_read_data_polling(void)
{
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  /* Initialize platform specific hardware */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lis2dw12_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LIS2DW12_ID) {
    printf("LIS2DW12 not found\r\n");
    return;  // exit instead of while(1)
  }

  /* Restore default configuration */
  lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);
  int reset_timeout = 0;
  do {
    lis2dw12_reset_get(&dev_ctx, &rst);
    HAL_Delay(10);
    reset_timeout++;
  } while (rst && reset_timeout < 50);

  /* Enable Block Data Update */
  lis2dw12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_2g);
  /* Configure filtering chain */
  lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT);
  lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_4);
  /* Configure power mode */
  lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_HIGH_PERFORMANCE);
  /* Set Output Data Rate */
  lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_25Hz);

  /* Wait for first measurement (25Hz = 40ms per sample) */
  HAL_Delay(100);

  /* Wait until data is ready with timeout */
  uint8_t reg;
  int timeout = 0;
  do {
    lis2dw12_flag_data_ready_get(&dev_ctx, &reg);
    HAL_Delay(10);
    timeout++;
  } while (!reg && timeout < 50);  // timeout after 500ms

  /* Read acceleration data */
  memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
  lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
    int32_t accel_mg_x_centi = ((int32_t)data_raw_acceleration[0] * 61);
    int32_t accel_mg_y_centi = ((int32_t)data_raw_acceleration[1] * 61);
    int32_t accel_mg_z_centi = ((int32_t)data_raw_acceleration[2] * 61);
    int32_t accel_mg_x_abs = (accel_mg_x_centi < 0) ? -accel_mg_x_centi : accel_mg_x_centi;
    int32_t accel_mg_y_abs = (accel_mg_y_centi < 0) ? -accel_mg_y_centi : accel_mg_y_centi;
    int32_t accel_mg_z_abs = (accel_mg_z_centi < 0) ? -accel_mg_z_centi : accel_mg_z_centi;
    printf("Acceleration [mg]:%ld.%02ld\t%ld.%02ld\t%ld.%02ld\r\n",
           (long)(accel_mg_x_centi / 100), (long)(accel_mg_x_abs % 100),
           (long)(accel_mg_y_centi / 100), (long)(accel_mg_y_abs % 100),
           (long)(accel_mg_z_centi / 100), (long)(accel_mg_z_abs % 100));
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
    HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_H, reg,
                      I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000);
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_H, reg,
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
