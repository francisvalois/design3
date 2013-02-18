################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../contrib/gtest/gtest-all.cc 

OBJS += \
./contrib/gtest/gtest-all.o 

CC_DEPS += \
./contrib/gtest/gtest-all.d 


# Each subdirectory must supply rules for building sources it contributes
contrib/gtest/%.o: ../contrib/gtest/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/olivier/design/design3/SudokubeSolver/contrib" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


