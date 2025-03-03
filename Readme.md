# ESP32C3 Display Project

Example using embassy to asynchronously use the i2c-bus and operate 2 devices in asynchronous mode:
- Pressure sensor ms5611 is read in one task and sends the data via 
- in another task, a button is evaluated. Each press increments a counter and sends it via a channel
- a task updates an ssd1306 display (128x32 pixels) by receiving the pressure sensor measurement and button counter via channels and displaying them

## Prerequisites

- Rust
- ESP-IDF
- An ESP32-C3 development board
- ms5611 with i2c interface  
- ssd1306 display with i2c interface
- I2C bus wiring: SDA: GPIO 6, SCL GPIO 7
- Button: Wiring GPIO2, GND

## Installation

1. Clone the repository:
  ```sh
  git clone https://github.com/your-username/esp32c3-display.git
  cd esp32c3-display
  ```

2. Install the dependencies:
  ```sh
  rustup target add riscv32imc-unknown-none-elf
  cargo install espflash
  ```

3. Build the project:
  ```sh
  cargo build
  ```

## Usage

1. Flash the firmware to the ESP32-C3 board:
  ```sh
  cargo run
  ```


2. Connect the display to the ESP32-C3 board according to the pinout in the documentation.2. Connect the display to the ESP32-C3 board according to the pinout in the documentation.

3. Start the board and check the display.3. Start the board and check the display.

## License## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
