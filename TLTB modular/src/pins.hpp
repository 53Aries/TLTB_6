// File Overview: Centralized ESP32-S3 pin map covering the TFT, encoder, rotary switch,
// INA226 sensors, relays, buzzer, and RF receiver connections.
// 2/20/2026 update pins for ESP board v1.5.2
#pragma once

// ======================= SPI (FSPI) for TFT =======================
#define PIN_FSPI_SCK    36 // SCL
#define PIN_FSPI_MOSI   37 // SDA
#define PIN_FSPI_MISO   36 // not used, but must be defined

// ======================= Display (ST7735S) =======================
#define PIN_TFT_CS      40
#define PIN_TFT_DC      39
#define PIN_TFT_RST     38
// TFT backlight not used - display runs without it
// #define PIN_TFT_BL      42  // repurposed for INA226 ALERT

// ======================= Rotary Encoder =======================
#define PIN_ENC_A       42
#define PIN_ENC_B       2
#define PIN_ENC_OK      1
#define PIN_ENC_BACK    43

// Encoder OK button active level (hardware drives HIGH when pressed)
#define ENC_OK_ACTIVE_LEVEL LOW

// ======================= Input Switches (active-LOW, external pull-ups) =======================
#define PIN_SW1_LH      4   // Left Turn
#define PIN_SW2_RH      5   // Right Turn
#define PIN_SW3_BRAKE   6   // Brake
#define PIN_SW4_TAIL    7   // Tail
#define PIN_SW5_MARK    15  // Marker
#define PIN_SW6_AUX     16  // Auxiliary
#define PIN_SW7_CYCLE   17  // Cycle

// ======================= I²C Bus (INA226 modules) =======================
#define PIN_I2C_SDA     47
#define PIN_I2C_SCL     48
// #Source INA bridge A0

// ======================= INA226 ALERT Pin =======================
// LOAD INA226 ALERT output for extreme overcurrent detection (>30A)
// Triggers before buck converter shutdown to log short circuit event
#define PIN_INA_LOAD_ALERT  41

// ======================= Buck Converter Enable =======================
#define PIN_BUCK_EN       14    // Active-high enable for buck converter

// ======================= Relays (ULN2803 active-high) =======================
#define PIN_RELAY_LH       8   // Left Turn
#define PIN_RELAY_RH       9   // Right Turn
#define PIN_RELAY_BRAKE   10   // Brake Lights
#define PIN_RELAY_TAIL    11   // Tail Lights
#define PIN_RELAY_MARKER  12   // Marker Lights
#define PIN_RELAY_AUX     13   // Auxiliary

// Array for DisplayUI.cpp relay status logic
static const int RELAY_PIN[] = {
  PIN_RELAY_LH,
  PIN_RELAY_RH,
  PIN_RELAY_BRAKE,
  PIN_RELAY_TAIL,
  PIN_RELAY_MARKER,
  PIN_RELAY_AUX
};

// ======================= Buzzer =======================
#define PIN_BUZZER      35

// ======================= RF (SYN480R Receiver) =======================
// DATA pin from SYN480R — must be level-shifted to 3.3 V
#define PIN_RF_DATA     21 
