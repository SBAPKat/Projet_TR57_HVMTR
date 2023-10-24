################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
C:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/cmd/DSP2833x_Headers_nonBIOS.cmd 

LIB_SRCS += \
C:/ti/controlSUITE/libs/math/IQmath/v15c/lib/IQmath_fpu32.lib \
C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/lib/rts2800_fpu32_fast_supplement.lib 

ASM_SRCS += \
../DLOG4CHC.asm \
C:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/source/DSP2833x_ADC_cal.asm \
C:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/source/DSP2833x_CodeStartBranch.asm \
../DSP2833x_usDelay.asm 

CMD_UPPER_SRCS += \
../F28335_RAM_HVPM_Sensorless.CMD 

C_SRCS += \
C:/ti/controlSUITE/device_support/f2833x/v142/DSP2833x_common/source/DSP2833x_ECan.c \
C:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/source/DSP2833x_GlobalVariableDefs.c \
C:/ti/controlSUITE/device_support/f2833x/v142/DSP2833x_common/source/DSP2833x_Sci.c \
../HVPM_TR57-DevInit_F2833x.c \
../HVPM_TR57.c 

C_DEPS += \
./DSP2833x_ECan.d \
./DSP2833x_GlobalVariableDefs.d \
./DSP2833x_Sci.d \
./HVPM_TR57-DevInit_F2833x.d \
./HVPM_TR57.d 

OBJS += \
./DLOG4CHC.obj \
./DSP2833x_ADC_cal.obj \
./DSP2833x_CodeStartBranch.obj \
./DSP2833x_ECan.obj \
./DSP2833x_GlobalVariableDefs.obj \
./DSP2833x_Sci.obj \
./DSP2833x_usDelay.obj \
./HVPM_TR57-DevInit_F2833x.obj \
./HVPM_TR57.obj 

ASM_DEPS += \
./DLOG4CHC.d \
./DSP2833x_ADC_cal.d \
./DSP2833x_CodeStartBranch.d \
./DSP2833x_usDelay.d 

OBJS__QUOTED += \
"DLOG4CHC.obj" \
"DSP2833x_ADC_cal.obj" \
"DSP2833x_CodeStartBranch.obj" \
"DSP2833x_ECan.obj" \
"DSP2833x_GlobalVariableDefs.obj" \
"DSP2833x_Sci.obj" \
"DSP2833x_usDelay.obj" \
"HVPM_TR57-DevInit_F2833x.obj" \
"HVPM_TR57.obj" 

C_DEPS__QUOTED += \
"DSP2833x_ECan.d" \
"DSP2833x_GlobalVariableDefs.d" \
"DSP2833x_Sci.d" \
"HVPM_TR57-DevInit_F2833x.d" \
"HVPM_TR57.d" 

ASM_DEPS__QUOTED += \
"DLOG4CHC.d" \
"DSP2833x_ADC_cal.d" \
"DSP2833x_CodeStartBranch.d" \
"DSP2833x_usDelay.d" 

ASM_SRCS__QUOTED += \
"../DLOG4CHC.asm" \
"C:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/source/DSP2833x_ADC_cal.asm" \
"C:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/source/DSP2833x_CodeStartBranch.asm" \
"../DSP2833x_usDelay.asm" 

C_SRCS__QUOTED += \
"C:/ti/controlSUITE/device_support/f2833x/v142/DSP2833x_common/source/DSP2833x_ECan.c" \
"C:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/source/DSP2833x_GlobalVariableDefs.c" \
"C:/ti/controlSUITE/device_support/f2833x/v142/DSP2833x_common/source/DSP2833x_Sci.c" \
"../HVPM_TR57-DevInit_F2833x.c" \
"../HVPM_TR57.c" 


