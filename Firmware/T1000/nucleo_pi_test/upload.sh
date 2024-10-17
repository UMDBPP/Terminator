openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "program Debug/nucleo_fram_test.elf verify reset"
