################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/ti/ccs1031/ccs/tools/compiler/gcc-arm-none-eabi-7-2017-q4-major-win32

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./bsp.o" \
"./dev_buzzer.o" \
"./dev_glcd.o" \
"./dev_mpu.o" \
"./dev_servomotor.o" \
"./dev_steppermotor.o" \
"./main.o" \
"./misc.o" \
"./php.o" \
"./php_adc.o" \
"./php_gpio.o" \
"./php_i2c.o" \
"./php_keypad.o" \
"./php_lcd.o" \
"./php_led.o" \
"./php_pwm.o" \
"./php_spi.o" \
"./php_switch.o" \
"./php_timer.o" \
"./php_uart.o" \
"./startup_tm4c_gnu.o" \
-Wl,-T"../tm4c123gh6pm.lds" \
$(GEN_CMDS__FLAG) \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
11_Dev_Nokia_LCD_Display.out \

EXE_OUTPUTS__QUOTED += \
"11_Dev_Nokia_LCD_Display.out" \

BIN_OUTPUTS += \
11_Dev_Nokia_LCD_Display.hex \

BIN_OUTPUTS__QUOTED += \
"11_Dev_Nokia_LCD_Display.hex" \


# All Target
all: $(OBJS) $(LDS_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "11_Dev_Nokia_LCD_Display.out"

# Tool invocations
11_Dev_Nokia_LCD_Display.out: $(OBJS) $(LDS_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: GNU Linker'
	"D:/ti/ccs1031/ccs/tools/compiler/gcc-arm-none-eabi-7-2017-q4-major-win32/bin/arm-none-eabi-gcc-7.2.1.exe" -DPART_TM4C123GH6PM -O0 -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -specs="nosys.specs" -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wl,-Map,"11_Dev_Nokia_LCD_Display.map" -o"11_Dev_Nokia_LCD_Display.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

11_Dev_Nokia_LCD_Display.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: GNU Objcopy Utility'
	"D:/ti/ccs1031/ccs/tools/compiler/gcc-arm-none-eabi-7-2017-q4-major-win32/bin/arm-none-eabi-objcopy.exe"  $(EXE_OUTPUTS__QUOTED) "11_Dev_Nokia_LCD_Display.hex"
	@echo 'Finished building secondary target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "bsp.o" "dev_buzzer.o" "dev_glcd.o" "dev_mpu.o" "dev_servomotor.o" "dev_steppermotor.o" "main.o" "misc.o" "php.o" "php_adc.o" "php_gpio.o" "php_i2c.o" "php_keypad.o" "php_lcd.o" "php_led.o" "php_pwm.o" "php_spi.o" "php_switch.o" "php_timer.o" "php_uart.o" "startup_tm4c_gnu.o" 
	-$(RM) "bsp.d" "dev_buzzer.d" "dev_glcd.d" "dev_mpu.d" "dev_servomotor.d" "dev_steppermotor.d" "main.d" "misc.d" "php.d" "php_adc.d" "php_gpio.d" "php_i2c.d" "php_keypad.d" "php_lcd.d" "php_led.d" "php_pwm.d" "php_spi.d" "php_switch.d" "php_timer.d" "php_uart.d" "startup_tm4c_gnu.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

