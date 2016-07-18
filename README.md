# STM32 Project
This is some projects based on the STM32 minimum development board
![some](/Doc/stm32_board_pinout.jpg?raw=true)

The board has STM32F103C8T6 (LQFP48) chip with 8 MHz external clock generator.

List of projects:
* [simple_printf](/simple_printf) 

The projects use [STM32Cube](http://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32cube.html?querycriteria=productId=SC2004):
* STM32Cube embedded software libraries
* STM32CubeMX configuration graphical wizard

The projects are built using [GNU ARM Embedded Toolchain](https://launchpad.net/gcc-arm-embedded).

For MCU programming you can use [ST-Link](https://github.com/texane/stlink) utility and [STLINKv2](https://www.adafruit.com/product/2548) USB programmer.
## How to setup the build tools
1. Download [STM32Cube libraries](http://www.st.com/content/st_com/en/products/embedded-software/mcus-embedded-software/stm32-embedded-software/stm32cube-embedded-software/stm32cubef1.html) (login required).
```bash
mkdir <stm32_tools>
cd <stm32_tools>
cp <download_dir>/en.stm32cubef1.zip .
unzip en.stm32cubef1.zip
wget https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q2-update/+download/gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2
tar jxf gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2
```
2. ST-Link utility setup instructions are [here](https://github.com/texane/stlink)





