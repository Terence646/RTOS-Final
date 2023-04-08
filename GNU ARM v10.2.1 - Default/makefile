################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/Source_Files/subdir.mk
-include src/subdir.mk
-include gecko_sdk_3.2.2/platform/service/udelay/src/subdir.mk
-include gecko_sdk_3.2.2/platform/service/system/src/subdir.mk
-include gecko_sdk_3.2.2/platform/service/sleeptimer/src/subdir.mk
-include gecko_sdk_3.2.2/platform/service/device_init/src/subdir.mk
-include gecko_sdk_3.2.2/platform/middleware/glib/glib/subdir.mk
-include gecko_sdk_3.2.2/platform/middleware/glib/dmd/display/subdir.mk
-include gecko_sdk_3.2.2/platform/micrium_os/ports/source/gnu/subdir.mk
-include gecko_sdk_3.2.2/platform/micrium_os/ports/source/generic/subdir.mk
-include gecko_sdk_3.2.2/platform/micrium_os/kernel/source/subdir.mk
-include gecko_sdk_3.2.2/platform/micrium_os/cpu/source/subdir.mk
-include gecko_sdk_3.2.2/platform/micrium_os/common/source/rtos/subdir.mk
-include gecko_sdk_3.2.2/platform/micrium_os/common/source/ring_buf/subdir.mk
-include gecko_sdk_3.2.2/platform/micrium_os/common/source/logging/subdir.mk
-include gecko_sdk_3.2.2/platform/micrium_os/common/source/lib/subdir.mk
-include gecko_sdk_3.2.2/platform/micrium_os/common/source/kal/subdir.mk
-include gecko_sdk_3.2.2/platform/emlib/src/subdir.mk
-include gecko_sdk_3.2.2/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_3.2.2/platform/common/src/subdir.mk
-include gecko_sdk_3.2.2/platform/Device/SiliconLabs/EFM32PG12B/Source/GCC/subdir.mk
-include gecko_sdk_3.2.2/platform/Device/SiliconLabs/EFM32PG12B/Source/subdir.mk
-include gecko_sdk_3.2.2/platform/CMSIS/RTOS2/Source/subdir.mk
-include gecko_sdk_3.2.2/hardware/driver/memlcd/src/memlcd_usart/subdir.mk
-include gecko_sdk_3.2.2/hardware/driver/memlcd/src/subdir.mk
-include gecko_sdk_3.2.2/hardware/board/src/subdir.mk
-include autogen/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Williams_RTOS_Final
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: main-build

# Main-build Target
main-build: Williams_RTOS_Final.axf

# Tool invocations
Williams_RTOS_Final.axf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "/Users/terencewilliams/SimplicityStudio/v5_workspace/Williams_RTOS_Final/autogen/linkerfile.ld" -Xlinker --gc-sections -Xlinker -Map="Williams_RTOS_Final.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o Williams_RTOS_Final.axf -Wl,--start-group "./autogen/sl_board_default_init.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_simple_button_instances.o" "./gecko_sdk_3.2.2/hardware/board/src/sl_board_control_gpio.o" "./gecko_sdk_3.2.2/hardware/board/src/sl_board_init.o" "./gecko_sdk_3.2.2/hardware/driver/memlcd/src/memlcd_usart/sl_memlcd_spi.o" "./gecko_sdk_3.2.2/hardware/driver/memlcd/src/sl_memlcd.o" "./gecko_sdk_3.2.2/hardware/driver/memlcd/src/sl_memlcd_display.o" "./gecko_sdk_3.2.2/platform/CMSIS/RTOS2/Source/os_systick.o" "./gecko_sdk_3.2.2/platform/Device/SiliconLabs/EFM32PG12B/Source/GCC/startup_efm32pg12b.o" "./gecko_sdk_3.2.2/platform/Device/SiliconLabs/EFM32PG12B/Source/system_efm32pg12b.o" "./gecko_sdk_3.2.2/platform/common/src/sl_slist.o" "./gecko_sdk_3.2.2/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_acmp.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_assert.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_cmu.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_core.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_emu.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_gpio.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_rtcc.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_system.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_usart.o" "./gecko_sdk_3.2.2/platform/micrium_os/common/source/kal/kal_kernel.o" "./gecko_sdk_3.2.2/platform/micrium_os/common/source/lib/lib_ascii.o" "./gecko_sdk_3.2.2/platform/micrium_os/common/source/lib/lib_mem.o" "./gecko_sdk_3.2.2/platform/micrium_os/common/source/lib/lib_str.o" "./gecko_sdk_3.2.2/platform/micrium_os/common/source/logging/logging.o" "./gecko_sdk_3.2.2/platform/micrium_os/common/source/ring_buf/ring_buf.o" "./gecko_sdk_3.2.2/platform/micrium_os/common/source/rtos/rtos_err_str.o" "./gecko_sdk_3.2.2/platform/micrium_os/common/source/rtos/rtos_utils.o" "./gecko_sdk_3.2.2/platform/micrium_os/cpu/source/cpu_core.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/cmsis_os2.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_cfg_app.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_core.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_dbg.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_flag.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_mon.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_msg.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_mutex.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_prio.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_q.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_sem.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_stat.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_task.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_time.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_tmr.o" "./gecko_sdk_3.2.2/platform/micrium_os/kernel/source/os_var.o" "./gecko_sdk_3.2.2/platform/micrium_os/ports/source/generic/arm_cpu_dwt_ts.o" "./gecko_sdk_3.2.2/platform/micrium_os/ports/source/gnu/armv7m_cpu_a.o" "./gecko_sdk_3.2.2/platform/micrium_os/ports/source/gnu/armv7m_os_cpu_a.o" "./gecko_sdk_3.2.2/platform/micrium_os/ports/source/gnu/armv7m_os_cpu_c.o" "./gecko_sdk_3.2.2/platform/middleware/glib/dmd/display/dmd_memlcd.o" "./gecko_sdk_3.2.2/platform/middleware/glib/glib/bmp.o" "./gecko_sdk_3.2.2/platform/middleware/glib/glib/glib.o" "./gecko_sdk_3.2.2/platform/middleware/glib/glib/glib_bitmap.o" "./gecko_sdk_3.2.2/platform/middleware/glib/glib/glib_circle.o" "./gecko_sdk_3.2.2/platform/middleware/glib/glib/glib_font_narrow_6x8.o" "./gecko_sdk_3.2.2/platform/middleware/glib/glib/glib_font_normal_8x8.o" "./gecko_sdk_3.2.2/platform/middleware/glib/glib/glib_font_number_16x20.o" "./gecko_sdk_3.2.2/platform/middleware/glib/glib/glib_line.o" "./gecko_sdk_3.2.2/platform/middleware/glib/glib/glib_polygon.o" "./gecko_sdk_3.2.2/platform/middleware/glib/glib/glib_rectangle.o" "./gecko_sdk_3.2.2/platform/middleware/glib/glib/glib_string.o" "./gecko_sdk_3.2.2/platform/service/device_init/src/sl_device_init_dcdc_s1.o" "./gecko_sdk_3.2.2/platform/service/device_init/src/sl_device_init_emu_s1.o" "./gecko_sdk_3.2.2/platform/service/device_init/src/sl_device_init_hfxo_s1.o" "./gecko_sdk_3.2.2/platform/service/device_init/src/sl_device_init_lfxo_s1.o" "./gecko_sdk_3.2.2/platform/service/device_init/src/sl_device_init_nvic.o" "./gecko_sdk_3.2.2/platform/service/sleeptimer/src/sl_sleeptimer.o" "./gecko_sdk_3.2.2/platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" "./gecko_sdk_3.2.2/platform/service/system/src/sl_system_init.o" "./gecko_sdk_3.2.2/platform/service/system/src/sl_system_kernel.o" "./gecko_sdk_3.2.2/platform/service/udelay/src/sl_udelay.o" "./gecko_sdk_3.2.2/platform/service/udelay/src/sl_udelay_armv6m_gcc.o" "./src/Source_Files/app.o" "./src/Source_Files/blink.o" "./src/Source_Files/bsp_os.o" "./src/Source_Files/capsense.o" "./src/Source_Files/cmu.o" "./src/Source_Files/fifo.o" "./src/Source_Files/gpio.o" "./src/Source_Files/memlcd_app.o" "./src/main.o" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: Williams_RTOS_Final.hex'
	arm-none-eabi-objcopy -O ihex "Williams_RTOS_Final.axf" "Williams_RTOS_Final.hex"
	@echo ' '

	@echo 'Building bin file: Williams_RTOS_Final.bin'
	arm-none-eabi-objcopy -O binary "Williams_RTOS_Final.axf" "Williams_RTOS_Final.bin"
	@echo ' '

	@echo 'Building s37 file: Williams_RTOS_Final.s37'
	arm-none-eabi-objcopy -O srec "Williams_RTOS_Final.axf" "Williams_RTOS_Final.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "Williams_RTOS_Final.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) Williams_RTOS_Final.axf
	-@echo ' '

.PHONY: all clean dependents main-build

-include ../makefile.targets
