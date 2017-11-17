ARCH= cortexm4f
TOOLCHAIN= arm-none-eabi
CCFLAGS+= -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS+= -ffunction-sections -fdata-sections
LDFLAGS+= -Xlinker --gc-sections -specs=nano.specs -specs=nosys.specs
