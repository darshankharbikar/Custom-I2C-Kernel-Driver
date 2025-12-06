# Custom-I2C-Kernel-Driver

Wrote low-level drivers without SDK support using TRM references and register maps.
Created interrupt-safe logging system with circular buffer.


Objective
Design and implement bare-metal GPIO and UART drivers for the STM32F411CEU6 microcontroller using direct register manipulation and technical reference manual documentation, 
ensuring reliable interrupt-driven communication and efficient data handling through a circular buffer–based logging system.


Components required
STM32F411CEU6 (Black Pill) board
USB to TTL serial converter (for UART debugging)
Micro-USB cable (for power and programming via ST-Link)
ST-Link V2 programmer/debugger
Breadboard and jumper wires
LEDs and 330 Ω resistors (for GPIO testing)
Push button (for GPIO input testing)
PC with STM32 toolchain (ARM-GCC, OpenOCD, and Makefile setup)

Connections
ST-Link V2 → STM32F411CEU6
SWDIO → SWDIO
SWCLK → SWCLK
GND → GND
3.3V → 3.3V
USB to TTL (UART debugging)
TX (USB-TTL) → PA3 (USART2_RX)
RX (USB-TTL) → PA2 (USART2_TX)
GND (USB-TTL) → GND (STM32)
GPIO Testing
LED anode → PA5 (GPIO output)
LED cathode → 330 Ω resistor → GND
Push button → PA0 (GPIO input)
Other terminal of button → 3.3 V (use internal pull-down or external 10 kΩ to GND)
	
Executions	
Connect STM32F411CEU6 to PC via ST-Link V2.
Verify connection using openocd -f interface/stlink.cfg -f target/stm32f4x.cfg.
Compile the project using make (ARM-GCC toolchain).
Flash the generated .elf or .bin file using OpenOCD or STM32CubeProgrammer CLI.
Open serial terminal (e.g., MobaXterm, PuTTY) at 115200 baud, 8N1 on the USB-TTL port.
Observe UART debug logs for GPIO state changes or circular buffer messages.
Press the button to trigger GPIO interrupt and check corresponding UART logs.
Toggle LED and confirm proper GPIO driver functionality.
Verify interrupt-safe circular buffer logging during continuous UART transmission.

Features:
Pure register access (no HAL).
Configures GPIOA for LED (PA5) and Button (PA0).
Initializes USART2 (PA2/PA3) at 115200 bps.
Implements basic circular buffer logging.
Demonstrates GPIO input/output and UART debug output.

Issues and their debugging	 
Checklist	
STM32F411CEU6 connected and powered correctly via ST-Link
 Toolchain installed: ARM-GCC, OpenOCD, Makefile configured
 stm32f411xe.h header available in include path
 Clock frequency confirmed (16 MHz HSI used for UART BRR value)
 GPIO connections verified: PA5 → LED, PA0 → Button
 UART connections verified: PA2 (TX) → USB-TTL RX, PA3 (RX) → USB-TTL TX
 Serial terminal configured to 115200 baud, 8N1
 Binary built successfully with make
 Program flashed successfully with OpenOCD or STM32CubeProgrammer
 “System Init OK” message visible on terminal
 Button press toggles LED and prints “Button Pressed”
 Circular buffer logs flush correctly without data loss
 Interrupts or polling verified as stable (no crashes or freezes) 
Artifacts
Source Code Files:
main.c (GPIO, UART, circular buffer implementation)
Makefile (build automation)
stm32f411xe.h (register definitions, CMSIS header)


Build Artifacts:
main.elf (for debugging)
main.bin (for flashing to MCU)
map file (for memory analysis)


Execution Proofs:
Serial terminal log showing “System Init OK” and “Button Pressed” messages
Video or photo showing LED toggling on button press
Screenshot of successful flash and OpenOCD output


Documentation:
TRM and datasheet references (sections used for GPIO and USART)
Pin connection diagram
Brief report summarizing design, testing, and observations




