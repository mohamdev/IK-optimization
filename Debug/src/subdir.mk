################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/generate-randPol-trajectories.cpp \
../src/polynom.cpp \
../src/utils.cpp 

OBJS += \
./src/generate-randPol-trajectories.o \
./src/polynom.o \
./src/utils.o 

CPP_DEPS += \
./src/generate-randPol-trajectories.d \
./src/polynom.d \
./src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/python2.7 -I/usr/include/x86_64-linux-gnu/python2.7 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


