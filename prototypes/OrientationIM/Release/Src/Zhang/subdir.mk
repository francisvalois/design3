################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Src/Zhang/CameraModel.cpp \
../Src/Zhang/DistorsionConverter.cpp \
../Src/Zhang/ExtrinsicParameters.cpp \
../Src/Zhang/IntrinsicParameters.cpp \
../Src/Zhang/Matrix.cpp \
../Src/Zhang/RotationConverter.cpp \
../Src/Zhang/Zhang.cpp 

OBJS += \
./Src/Zhang/CameraModel.o \
./Src/Zhang/DistorsionConverter.o \
./Src/Zhang/ExtrinsicParameters.o \
./Src/Zhang/IntrinsicParameters.o \
./Src/Zhang/Matrix.o \
./Src/Zhang/RotationConverter.o \
./Src/Zhang/Zhang.o 

CPP_DEPS += \
./Src/Zhang/CameraModel.d \
./Src/Zhang/DistorsionConverter.d \
./Src/Zhang/ExtrinsicParameters.d \
./Src/Zhang/IntrinsicParameters.d \
./Src/Zhang/Matrix.d \
./Src/Zhang/RotationConverter.d \
./Src/Zhang/Zhang.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Zhang/%.o: ../Src/Zhang/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


