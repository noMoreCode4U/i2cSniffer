# STM32 I2C Sniffer (Register-Level)

This project provides a bare-metal **I2C Sniffer** for STM32 microcontrollers. It passively monitors I2C communication on specific GPIO pins, decodes the bus traffic (Start, Stop, Addresses, Data, ACK/NACKs), and prints the decoded information to a UART terminal. This is particularly useful for debugging I2C peripheral interactions without interfering with the bus.

---

## Features

* **Passive I2C Monitoring:** Non-intrusively listens to I2C bus traffic.
* **Register-Level Control:** GPIO and EXTI configurations are done at the register level for fine-grained control and understanding.
* **Interrupt-Driven:** Leverages external interrupts (EXTI) on the SCL line to capture I2C clock edges for precise bit-banging and decoding.
* **State Machine Decoding:** Implements a robust state machine to accurately parse I2C transactions including:
    * Start and Stop Conditions
    * 7-bit Slave Addresses (with Read/Write direction)
    * ACK/NACK signals
    * Data Bytes (identifies register addresses and subsequent data for write operations)
* **UART Output:** Decoded I2C frames are sent over UART for easy viewing on a serial terminal.
* **STM32F4xx Compatibility:** Designed with STM32F4 series in mind (e.g., `stm32f4xx.h`, `STM32F407VGTX_FLASH.ld`).

---

## Hardware Requirements

* **STM32 Microcontroller:** An STM32 board (e.g., Nucleo, Discovery, custom board).
* **I2C Bus:** The I2C bus you want to sniff.
* **UART Connection:** A serial-to-USB converter or built-in ST-LINK V2-1 virtual COM port for viewing output.

---

## Pin Configuration

The sniffer is configured to use the following default pins, which can be modified in `main.c` (under `Private define` section):

* **SCL (Serial Clock Line):** **GPIOB Pin 6** (`PB6`)
* **SDA (Serial Data Line):** **GPIOB Pin 7** (`PB7`)

These pins are configured as **input with pull-ups** to correctly observe the open-drain I2C bus.

---

## Software Requirements

* **STM32CubeIDE:** Recommended IDE for building and flashing this project.
* **STM32CubeF4 Firmware Package:** Necessary for HAL libraries and device definitions.

---

## How It Works

The I2C Sniffer operates based on an interrupt-driven state machine:

1.  **GPIO & EXTI Setup:**
    * PB6 (SCL) and PB7 (SDA) are configured as input with internal pull-ups at the register level.
    * An external interrupt (EXTI) is enabled for PB6 (SCL) on **both rising and falling edges**. This allows the microcontroller to detect every clock transition.
2.  **Interrupt Service Routine (ISR):**
    * The `EXTI9_5_IRQHandler` is triggered on every SCL edge.
    * Inside the ISR, the current state of SCL and SDA lines is read.
    * **Start Condition:** Detected when SDA goes low while SCL is high. This resets the state machine.
    * **Stop Condition:** Detected when SDA goes high while SCL is high. This also resets the state machine.
    * **Data Decoding:** Between Start and Stop conditions, a state machine tracks the I2C transaction flow:
        * **Address Byte:** The first 8 bits after a Start condition are decoded as the 7-bit slave address and the R/W bit.
        * **ACK/NACK:** The 9th bit after an address or data byte is monitored for ACK (0) or NACK (1).
        * **Data Bytes:** Subsequent 8-bit sequences are decoded as data. For write transactions, the first data byte is often interpreted as a register address, followed by the data written to that register. For read transactions, these are the bytes sent by the slave.
3.  **UART Output:** All decoded events and data are formatted into strings and sent via `UART_Print` to `USART2`, which can be viewed using any serial terminal program.

---

## Getting Started

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/noMoreCode4U/i2cSniffer.git](https://github.com/noMoreCode4U/i2cSniffer.git)
    cd i2cSniffer
    ```
    *(Note: The provided code snippet implies it's part of an STM32CubeIDE project. You might need to integrate this code into a new or existing CubeIDE project.)*
2.  **Open in STM32CubeIDE:**
    * If starting a new project: Create a new STM32 project for your target MCU (e.g., STM32F407VG).
    * Copy the provided C code into your `main.c` file.
    * Ensure the CubeMX generated code for `MX_GPIO_Init()` and `MX_USART2_UART_Init()` is present and correctly configures `USART2` (e.g., `PA2` for TX, `PA3` for RX).
    * Disable CubeMX's default configuration for `PB6` and `PB7` if it conflicts with the register-level setup.
3.  **Configure System Clock:** Ensure `SystemClock_Config()` is set up for your board.
4.  **Build the project:** Compile the code.
5.  **Flash to STM32:** Upload the compiled firmware to your STM32 board.
6.  **Connect Serial Terminal:** Connect your PC to the STM32's UART (e.g., via the ST-LINK V2-1 virtual COM port on Nucleo boards) with a baud rate configured in your `MX_USART2_UART_Init()`.
7.  **Observe Output:** As I2C communication occurs on PB6/PB7, you will see the decoded transactions printed in your serial terminal.

---

## Customization

* **I2C Pins:** Modify `I2C_SCL_PORT`, `I2C_SCL_PIN`, `I2C_SDA_PORT`, `I2C_SDA_PIN` definitions if you wish to monitor different I2C lines. Remember to update the corresponding EXTI line selection in `I2C_Sniffer_GPIO_EXTI_Init()` if the SCL pin changes.
* **UART:** Adjust `huart2` settings in `MX_USART2_UART_Init()` for a different UART peripheral or baud rate.

---

## Contribution

Contributions are welcome! If you have improvements, bug fixes, or new features (e.g., 10-bit address support, clock stretching detection), feel free to fork the repository and submit a pull request.

---

## License

(Please add your desired license information here, e.g., MIT, Apache 2.0, GPL, etc.)

---
**Disclaimer:** This code is provided as an example for I2C sniffing at the register level. It might require adjustments based on your specific STM32 microcontroller and hardware setup.
