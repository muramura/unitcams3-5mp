#pragma once

#include "driver/gpio.h"

// ==========================================
// Unit CamS3-5MP Pin Definitions
// ==========================================

// --- StampFly FC Connection (UART via Grove Port) ---
// Connected to StampFly Grove Red connector (J3) via straight Grove-to-SH cable
// Pin 1: CamS3 TX (GPIO 19) <---> StampFly RX (Pin 1, GPIO 15)
// Pin 2: CamS3 RX (GPIO 20) <---> StampFly TX (Pin 2, GPIO 13)
// Pin 3: CamS3 5V            <---> StampFly 5V (Pin 3)
// Pin 4: CamS3 GND           <---> StampFly GND (Pin 4)
#define BOARD_PIN_UART_TX       GPIO_NUM_19   // Grove Pin 1 (TX to StampFly RX)
#define BOARD_PIN_UART_RX       GPIO_NUM_20   // Grove Pin 2 (RX from StampFly TX)
#define BOARD_UART_PORT         UART_NUM_1
#define BOARD_UART_BAUDRATE     2000000        // 2,000,000 bps (2Mbps)

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
