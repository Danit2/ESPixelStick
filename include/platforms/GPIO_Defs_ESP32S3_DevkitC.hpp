#pragma once
/*
* GPIO_Defs_ESP32S3_DevkitC.hpp - Output Management class
*
* Project: ESPixelStick - An ESP8266 / ESP32 and E1.31 based pixel driver
* Copyright (c) 2025-2026 Shelby Merrick
* http://www.forkineye.com
*
*  This program is provided free for you to use in any way that you wish,
*  subject to the laws and regulations where you are using it.  Due diligence
*  is strongly suggested before using this code.  Please give credit where due.
*
*  The Author makes no warranty of any kind, express or implied, with regard
*  to this program or the documentation contained in this document.  The
*  Author shall not be liable in any event for incidental or consequential
*  damages in connection with, or arising out of, the furnishing, performance
*  or use of these programs.
*
*/

//Output Manager
#define MAX_NUM_RMT_CHANNELS 4
#define DEFAULT_RMT_0_GPIO  gpio_num_t::GPIO_NUM_2
#define DEFAULT_RMT_1_GPIO  gpio_num_t::GPIO_NUM_42
#define DEFAULT_RMT_2_GPIO  gpio_num_t::GPIO_NUM_41
#define DEFAULT_RMT_3_GPIO  gpio_num_t::GPIO_NUM_40

// SPI Output
// #define SUPPORT_SPI_OUTPUT
// #define DEFAULT_SPI_DATA_GPIO   gpio_num_t::GPIO_NUM_26
// #define DEFAULT_SPI_CLOCK_GPIO  gpio_num_t::GPIO_NUM_33
// #define DEFAULT_SPI_CS_GPIO     gpio_num_t::GPIO_NUM_32
// #define DEFAULT_SPI_DEVICE      SPI2_HOST

#define DEFAULT_I2C_SDA         gpio_num_t::GPIO_NUM_17
#define DEFAULT_I2C_SCL         gpio_num_t::GPIO_NUM_16

// File Manager
#define SUPPORT_SD
#define ENABLE_DEDICATED_SPI 1
#define SD_CARD_MISO_PIN        gpio_num_t::GPIO_NUM_7
#define SD_CARD_MOSI_PIN        gpio_num_t::GPIO_NUM_6
#define SD_CARD_CLK_PIN         gpio_num_t::GPIO_NUM_5
#define SD_CARD_CS_PIN          gpio_num_t::GPIO_NUM_4

#define DEFAULT_RELAY_GPIO      gpio_num_t::GPIO_NUM_15

// Special settings for the ESP32S3 Processors
#define DEFAULT_CONSOLE_TX_GPIO gpio_num_t::GPIO_NUM_43
#define DEFAULT_CONSOLE_RX_GPIO gpio_num_t::GPIO_NUM_44

// Output Types
// Not Finished - #define SUPPORT_OutputType_TLS3001
// #define SUPPORT_OutputType_APA102           // SPI
#define SUPPORT_OutputType_DMX              // UART / RMT
#define SUPPORT_OutputType_GECE             // UART / RMT
// #define SUPPORT_OutputType_GRINCH           // SPI
#define SUPPORT_OutputType_GS8208           // UART / RMT
#define SUPPORT_OutputType_Renard           // UART / RMT
#define SUPPORT_OutputType_Serial           // UART / RMT
#define SUPPORT_OutputType_TM1814           // UART / RMT
#define SUPPORT_OutputType_UCS1903          // UART / RMT
#define SUPPORT_OutputType_UCS8903          // UART / RMT
// #define SUPPORT_OutputType_WS2801           // SPI
#define SUPPORT_OutputType_WS2811           // UART / RMT
#define SUPPORT_OutputType_Relay            // GPIO
#define SUPPORT_OutputType_Servo_PCA9685    // I2C (default pins)
#define SUPPORT_OutputType_FireGod          // UART / RMT
