#-c significa que NO linkemos el modulo que generemos.
CC=arm-none-eabi-gcc
MACH=cortex-m4
CFLAGS= -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -O0 -Iinclude/
LSFLAGS= -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nano.specs -T stm32_ls.ld -Wl,-Map=final_mem_map.map
LSFLAGS_SH = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=rdimon.specs -T stm32_ls.ld -Wl,-Map=final_mem_map.map

#semi: main.o stm32f407xx_gpio_driver.o

all: main.o stm32f407xx_gpio_driver.o stm32_startup.o syscalls.o final.elf

main.o: main.c
	$(CC) $(CFLAGS) $^ -o $@

stm32f407xx_gpio_driver.o: source/stm32f407xx_gpio_driver.c
	$(CC) $(CFLAGS) $^ -o $@

stm32_startup.o: source/start_stm32.c
	$(CC) $(CFLAGS) -o $@ $^

syscalls.o: source/syscalls.c
	$(CC) $(CFLAGS) -o $@ $^

final.elf: main.o stm32f407xx_gpio_driver.o stm32_startup.o syscalls.o
	$(CC) $(LSFLAGS) -o $@ $^

clean:
	rm -rf *.o *.elf
