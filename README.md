# STM32F407 Bare-Metal Driver

本專案以 **STM32F407** 為開發版，實作 Bare-metal Peripheral Driver。

主要透過直接操作 MCU Register，理解 GPIO、SPI、I2C、Interrupt 等周邊功能的底層運作，並將 Register 操作封裝成 Driver API，而非完全依賴原生 STM32 HAL Library。

---

## 開發環境

- MCU：STM32F407
- Board：STM32F407 Discovery
- IDE：STM32CubeIDE
- Language：C
- Architecture：ARM Cortex-M4

---

## GPIO Driver

實作 GPIO 基本控制與 Interrupt 功能：

- Peripheral Clock Enable / Disable
- GPIO Init / DeInit
- Input / Output Mode
- Pull-up / Pull-down
- Output Type
- Output Speed
- Alternate Function
- Pin / Port Read & Write
- GPIO Toggle
- EXTI Interrupt
- NVIC Interrupt Enable
- Interrupt Priority

透過 GPIO Driver 練習：

```text
RCC
 ↓
GPIO Register
 ↓
EXTI
 ↓
NVIC
 ↓
ISR
```

---

## SPI Driver

實作 SPI Communication Driver，包含：

- Master / Slave Mode
- Full-duplex / Half-duplex
- Baud Rate Prescaler
- CPOL / CPHA
- 8-bit / 16-bit Data Frame
- Software Slave Management
- Blocking Transmission
- Interrupt-based TX / RX
- TXE / RXNE / OVR Interrupt Handling
- Application Callback

藉此理解 SPI 的 Clock、Data Sampling 與 Interrupt-driven Communication。

```text
Application
    │
    ▼
SPI Driver
    │
    ▼
SPI Peripheral
    │
    ├── SCLK
    ├── MOSI
    ├── MISO
    └── NSS
```

---

## I2C Driver

目前建立 I2C Driver 基礎架構，包括：

- Peripheral Clock Control
- ACK Control
- 基礎 Initialization
- Master TX 測試程式

> I2C Driver 目前仍持續開發中，尚未完成完整的 Master TX / RX API。

---

## Example Applications

| File | Description |
|---|---|
| `001_LED_toggle.c` | GPIO Output / LED Toggle |
| `002_LED_Button.c` | GPIO Button Input |
| `005button_interrupt.c` | External Interrupt |
| `006_tx_testing.c` | SPI Transmission |
| `007spi_txonly_arduino.c` | STM32 ↔ Arduino SPI 測試 |
| `010i2c_master_tx_testing.c` | I2C Master TX 測試 |

---

## Repository Structure

```text
Stm32_Driver/
│
├── drivers/
│   ├── Inc/
│   │   ├── stm32f407.h
│   │   ├── stm32f407_gpio_driver.h
│   │   ├── stm32f407_spi_driver.h
│   │   └── stm32f407_i2c_driver.h
│   │
│   └── Src/
│       ├── stm32f407_gpio_driver.c
│       ├── stm32f407_spi_driver.c
│       └── stm32f407_i2c_driver.c
│
├── Src/
├── Startup/
└── STM32F407VGTX_FLASH.ld
```

---

## 實作重點

- Memory-mapped I/O
- Register-level Programming
- Bit Manipulation
- GPIO / Alternate Function
- SPI Protocol
- I2C Protocol
