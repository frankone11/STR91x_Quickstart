CFLAGS := -Wextra -D_STR9x_ -D_STR91x_
COMPILER_TYPE := arm-none-eabi
CC := $(COMPILER_TYPE)-gcc
MCUBASE := /home/paco/Antiguos/ejecutables/Ride7/ARM/
INCLUDE_PATHS := -I$(MCUBASE)/include/ -I$(MCUBASE)/STR91x_Lib/include/ -I$(MCUBASE)/STR91x_Lib/ -I$(MCUBASE)/e_stdio/src/
TARGET := QuickStart_STR9
LINKFLAGS := -mcpu=arm966e-s -specs=nano.specs -specs=nosys.specs -T $(TARGET).elf.ld -T $(MCUBASE)/STR91x_COMMON.ld -Wl,-Map -Xlinker $(TARGET).map -N -nostartfiles
OBJS := main.o syscalls.o write.o sbrk.o uart.o
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size


.PHONY=all clean menu

all: $(TARGET)

$(TARGET):$(OBJS)
	$(CC) $(LINKFLAGS) -o $@ $^
	$(OBJCOPY) $(TARGET) --target=ihex $(TARGET).hex
	$(SIZE) $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c $< $(INCLUDE_PATHS)

clean:
	@rm -f $(TARGET) $(OBJS) $(TARGET).hex $(TARGET).map

menu:
	@echo No hay menu.