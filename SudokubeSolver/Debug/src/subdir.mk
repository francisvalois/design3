################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Case.cpp \
../src/Sudokube.cpp \
../src/SudokubeSolver.cpp 

OBJS += \
./src/Case.o \
./src/Sudokube.o \
./src/SudokubeSolver.o 

CPP_DEPS += \
./src/Case.d \
./src/Sudokube.d \
./src/SudokubeSolver.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


