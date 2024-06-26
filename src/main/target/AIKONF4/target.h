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

#define BOARD_NAME        AIKONF4
#define MANUFACTURER_ID   AIKO
#define TARGET_BOARD_IDENTIFIER "S405"  // generic ID
#define FC_TARGET_MCU     STM32F405     // not used in EmuF

#define LED0_PIN                PB4
#define USE_BEEPER
#define BEEPER_PIN              PB5
#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR       true

#define INVERTER_PIN_UART1      PC0

#define CAMERA_CONTROL_PIN      PB3

#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_ICM20602
#define USE_GYRO_SPI_ICM20602
#define USE_ACCGYRO_BMI270
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6500

#define MPU6000_CS_PIN          SPI1_NSS_PIN
#define MPU6000_SPI_INSTANCE    SPI1
#define GYRO_MPU6000_ALIGN      CW0_DEG
#define ACC_MPU6000_ALIGN       CW0_DEG

// ICM2060x detected by MPU6500 driver
#define MPU6500_CS_PIN          MPU6000_CS_PIN
#define MPU6500_SPI_INSTANCE    MPU6000_SPI_INSTANCE
#define GYRO_MPU6500_ALIGN      GYRO_MPU6000_ALIGN
#define ACC_MPU6500_ALIGN       ACC_MPU6000_ALIGN

#define ACC_BMI270_ALIGN         CW0_DEG
#define GYRO_BMI270_ALIGN        CW0_DEG
#define BMI270_CS_PIN            PA4
#define BMI270_SPI_INSTANCE      SPI1

#define USE_BARO
#define USE_BARO_BMP280

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      SPI3_NSS_PIN
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            SPI2_NSS_PIN
#define FLASH_SPI_INSTANCE      SPI2

#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          PD2

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_AHB1_PERIPHERALS  RCC_AHB1Periph_DMA2

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_TX_PIN            PA0

#define USE_SOFTSERIAL1
#define SOFTSERIAL1_TX_PIN      PA9

#define USE_SOFTSERIAL2
#define SOFTSERIAL2_RX_PIN      PA1

#define SERIAL_PORT_COUNT       7 // VCP, USART1, USART3, USART4, USART6, SOFT_SERIAL1, SOFT_SERIAL2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB6  // (HARDARE=0,PPM)

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define USE_ADC
#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_ADC_PIN            PC2
#define RSSI_ADC_PIN            PC3
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART1

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        ( FEATURE_OSD | FEATURE_SOFTSERIAL )

#define TARGET_IO_PORTA         ( BIT(15) | BIT(12) | BIT(11) | BIT(10) | BIT(9) | BIT(7) | BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0) )
#define TARGET_IO_PORTB         ( BIT(15) | BIT(14) | BIT(13) | BIT(12) | BIT(11) | BIT(10) | BIT(9) | BIT(8) | BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(1) | BIT(0) )
#define TARGET_IO_PORTC         ( BIT(12) | BIT(11) | BIT(10) | BIT(9) | BIT(8) | BIT(7) | BIT(6) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0) )
#define TARGET_IO_PORTD         ( BIT(2) )

#define USABLE_TIMER_CHANNEL_COUNT 16
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) )
