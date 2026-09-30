#pragma once

#include "driver/gpio.h"

// ==========================================
// Unit CamS3-5MP Pin Definitions
// ==========================================

// --- StampFly FC Connection (UART) ---
#define BOARD_PIN_UART_TX       GPIO_NUM_43   // Connected to StampFly RX
#define BOARD_PIN_UART_RX       GPIO_NUM_44   // Connected to StampFly TX
#define BOARD_UART_PORT         UART_NUM_1
#define BOARD_UART_BAUDRATE     115200        // Default ArduPilot SERIAL3 baudrate

// --- Onboard LED & Button ---
#define BOARD_PIN_LED           GPIO_NUM_14   // Blue LED (Active LOW in typical M5Stack circuits)
#define BOARD_PIN_BUTTON_A      GPIO_NUM_0    // BOOT Button

// --- Camera (PY260 / DVP Interface) ---
#define CAMERA_PIN_PWDN         -1
#define CAMERA_PIN_RESET        21
#define CAMERA_PIN_XCLK         11
#define CAMERA_PIN_SIOD         17
#define CAMERA_PIN_SIOC         41

#define CAMERA_PIN_D7           13
#define CAMERA_PIN_D6           4
#define CAMERA_PIN_D5           10
#define CAMERA_PIN_D4           5
#define CAMERA_PIN_D3           7
#define CAMERA_PIN_D2           16
#define CAMERA_PIN_D1           15
#define CAMERA_PIN_D0           6

#define CAMERA_PIN_VSYNC        42
#define CAMERA_PIN_HREF         18
#define CAMERA_PIN_PCLK         12

#define BOARD_XCLK_FREQ_HZ      20000000

// --- MicroSD Card (SPI) ---
#define BOARD_PIN_SD_CS         GPIO_NUM_9
#define BOARD_PIN_SD_MOSI       GPIO_NUM_38
#define BOARD_PIN_SD_CLK        GPIO_NUM_39
#define BOARD_PIN_SD_MISO       GPIO_NUM_40

// --- Digital Microphone ---
#define BOARD_PIN_MIC_CLK       GPIO_NUM_47
#define BOARD_PIN_MIC_DATA      GPIO_NUM_48
