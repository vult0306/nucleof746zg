Nucleo-F746ZG Project

About
Hardware: Nucleo-F746ZG board
IC: STM32F746ZG
IDE: STM32CUBEIDE (and Rust)


Application description:
1: dma_uart
    Make the nucleo-f746zg as a chatting device. Take user's message from keyboard,
    send this message over the internet. FreeRTOS, UART-DMA supported.

2: spi_loopback
    Test the SPI functionality by sending out message from MOSI pin and read it back
    via MISO.

3: blinky_rust
    Blinky LED using Rust


Getting started
1. Clone this repository in to your directory
2. For STM32CUBE project, make sure you have STM32CUBE ide installed. Then you can
    simply open the project managerment file.
3. For Rust project, please follow the following commands to run the applications:
    - Go to rust application directory:
    $ cd blinky_rust
    
    - Compile the application:
    $ Cargo build --example blinky --features="stm32f746 rt"

    - Attact the micro-USB cable to your board
    - Flash the application:
    $ Cargo flash --chip stm32f746zg --example blinky --feature="stm32f746 rt"

