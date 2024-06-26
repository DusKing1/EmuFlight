/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#define TARGET_BOARD_IDENTIFIER "IFRC"
#define USBD_PRODUCT_STRING "JBF7_DJI"

#define USE_TARGET_CONFIG

#define ENABLE_DSHOT_DMAR       true

#define USE_BEEPER
#define BEEPER_PIN              PC15
#define BEEPER_INVERTED

#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PC10

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_SOFTSERIAL1

#define SERIAL_PORT_COUNT       7 //VCP, USART1, USART2,USART3,USART4,USART5,USART6

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C2_SCL                PB10
#define I2C2_SDA                PB11
#define I2C_DEVICE              (I2CDEV_2)

#define LED0_PIN                PC4

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define CAMERA_CONTROL_PIN      PA0

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define CURRENT_METER_ADC_PIN   PC2
#define VBAT_ADC_PIN            PC1
#define RSSI_ADC_PIN            PC0
#define CURRENT_METER_SCALE_DEFAULT 100
//#define VBAT_SCALE              100

#define USE_SDCARD
#define SDCARD_DETECT_INVERTED
//#define SDCARD_DETECT_PIN       PA4
#define SDCARD_SPI_INSTANCE     SPI3
#define SDCARD_SPI_CS_PIN       PA4

#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 422kHz
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER      8 // 27MHz
//#define SDCARD_DMA_STREAM_TX_FULL             DMA2_Stream5
//#define SDCARD_DMA_CHANNEL                    3

#define USE_PINIO
#define PINIO1_PIN              PC13
#define PINIO2_PIN              PC14
#define PINIO3_PIN              PB8
#define USE_PINIOBOX

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PB9
#define FLASH_SPI_INSTANCE      SPI3

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_DUAL_GYRO
#define USE_MPU_DATA_READY_SIGNAL

#define USE_EXTI
#define GYRO_1_EXTI_PIN         PA8
#define GYRO_2_EXTI_PIN         PB2
#define MPU_INT_EXTI

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC
#define USE_ACC_SPI_MPU6500

#define GYRO_1_CS_PIN           PA15
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_2_CS_PIN           PC3
#define GYRO_2_SPI_INSTANCE     SPI1

#define ACC_MPU6500_1_ALIGN     CW0_DEG
#define GYRO_MPU6500_1_ALIGN    CW0_DEG
#define ACC_MPU6500_2_ALIGN     CW90_DEG
#define GYRO_MPU6500_2_ALIGN    CW90_DEG
#define GYRO_1_ALIGN            GYRO_MPU6500_1_ALIGN
#define GYRO_2_ALIGN            GYRO_MPU6500_2_ALIGN
#define ACC_1_ALIGN             ACC_MPU6500_1_ALIGN
#define ACC_2_ALIGN             ACC_MPU6500_2_ALIGN

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_BOTH

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_UART5
#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY )

#define USE_GPS

#define USE_BARO
#define USE_BARO_BMP280
#define BARO_I2C_INSTANCE     I2CDEV_2

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE      I2CDEV_2

#define SENSORS_SET (SENSOR_ACC | SENSOR_BARO)

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT  11
#define USED_TIMERS ( TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8)| TIM_N(9) )
