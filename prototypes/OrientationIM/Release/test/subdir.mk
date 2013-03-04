################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/buffer.cpp \
../test/edgel.cpp \
../test/edgeldetector.cpp \
../test/linesegment.cpp \
../test/main.cpp 

OBJS += \
./test/buffer.o \
./test/edgel.o \
./test/edgeldetector.o \
./test/linesegment.o \
./test/main.o 

CPP_DEPS += \
./test/buffer.d \
./test/edgel.d \
./test/edgeldetector.d \
./test/linesegment.d \
./test/main.d 


# Each subdirectory must supply rules for building sources it contributes
test/%.o: ../test/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


