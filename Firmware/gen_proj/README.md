This project was originally generated by the STM32 Cube IDE. I just stole it and rippped it apart so I could build it myself and not be forced to use their IDE. Hopefully I'll also be able to steal some of their HAL functions too since I don't want to have to write them myself. The original project can be found in "empty_test_proj.zip"

# Project Structure
The structure of the project isn't too complicated.
- **build**: this fold contains the makefile and linker script used to build the final .elf file and other artifacts created during a build
- **src**: this contains source files that YOU write, things like main.c and other small or libraries
- **lib**: this is where you should put libraries you need to use for things like accessing I2C devices or parsing GPS data
- **vendor**: this includes code written by ARM and ST which perform basic configuration functions (e.g. the basic assembly run by the processor at startup that puts your application code in the right place in memory), unless you know what you are doing in here don't touch

# References
Things I used to figure this out.
- ENEE440 - great class IMO, but I'm really into all this stuff so YMMV
- https://kleinembedded.com/stm32-without-cubeide-part-1-the-bare-necessities/
    - this really does explain most of it quite nicely, I had to figure some things out on my own to get this approach working with ST's provided startup and linker scripts
- https://vivonomicon.com/2018/04/02/bare-metal-stm32-programming-part-1-hello-arm/