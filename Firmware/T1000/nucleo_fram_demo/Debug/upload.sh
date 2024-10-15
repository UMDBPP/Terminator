openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "program test.elf verify reset"
