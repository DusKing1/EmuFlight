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

//#define USE_TARGET_CONFIG

#define BOARD_NAME        XILOF4
#define MANUFACTURER_ID   GEFP
#define TARGET_BOARD_IDENTIFIER "S405"  // generic ID
#define FC_TARGET_MCU     STM32F405     // not used in EmuF

#define LED0_PIN                PC15
#define USE_BEEPER
#define BEEPER_PIN              PC5
#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR       true

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PC4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define USE_ACCGYRO_BMI270
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500

#define USE_SPI_GYRO
#define USE_MPU_DATA_READY_SIGNAL

#define ACC_MPU6000_ALIGN        CW180_DEG
#define GYRO_MPU6000_ALIGN       CW180_DEG
#define MPU6000_CS_PIN           PC4
#define MPU6000_SPI_INSTANCE     SPI1

#define ACC_MPU6500_ALIGN        CW180_DEG
#define GYRO_MPU6500_ALIGN       CW180_DEG
#define MPU6500_CS_PIN           PC4
#define MPU6500_SPI_INSTANCE     SPI1

#define ACC_BMI270_ALIGN         CW180_DEG
#define GYRO_BMI270_ALIGN        CW180_DEG
#define BMI270_CS_PIN            PC4
#define BMI270_SPI_INSTANCE      SPI1


// *************** Flash **************************
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PB2
#define FLASH_SPI_INSTANCE      SPI2
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

// *************** OSD *****************************
#define USE_SPI
#define USE_SPI_DEVICE_3

#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PD2

// *************** UART *****************************
#define USE_VCP
#define USE_USB_DETECT


#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3

#define USE_UART3
#define UART3_RX_PIN            PC11
#define UART3_TX_PIN            PC10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       7

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART1

// *************** PIN *****************************

#define PINIO1_PIN             PB1

// *************** I2C *****************************

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

// *************** ADC *****************************
#define USE_ADC
#define ADC3_DMA_OPT            0
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
#define RSSI_ADC_PIN            PC2

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY )
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 179


#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA3

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff

#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS             (TIM_N(1)|TIM_N(2)|TIM_N(4)|TIM_N(8))
