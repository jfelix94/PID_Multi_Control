################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/ti/ccsv8/tools/compiler/gcc-arm-none-eabi-7-2017-q4-major-win32/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"C:/Users/jfeli/workspace_v8/PID_Multi_TFG" -I"C:/ti/simplelink_msp432p4_sdk_2_20_00_12/source" -I"C:/ti/simplelink_msp432p4_sdk_2_20_00_12/source/third_party/CMSIS/Include" -I"C:/ti/simplelink_msp432p4_sdk_2_20_00_12/source/ti/posix/gcc" -I"D:/Documents/Unifei/TFG/FreeRTOS/FreeRTOSv10.0.1/FreeRTOS/Source/include" -I"D:/Documents/Unifei/TFG/FreeRTOS/FreeRTOSv10.0.1/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/jfeli/workspace_v8/freertos_builds_MSP_EXP432P401R_release_gcc" -I"C:/ti/ccsv8/tools/compiler/gcc-arm-none-eabi-7-2017-q4-major-win32/arm-none-eabi/include/newlib-nano" -I"C:/ti/ccsv8/tools/compiler/gcc-arm-none-eabi-7-2017-q4-major-win32/arm-none-eabi/include" -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


