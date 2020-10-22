################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/generate-randPol-trajectories.cpp \
../src/hs071_main.cpp \
../src/hs071_nlp.cpp \
../src/kinematics.cpp \
../src/polynom.cpp \
../src/test-pinocchio-fkine.cpp \
../src/utils.cpp 

OBJS += \
./src/generate-randPol-trajectories.o \
./src/hs071_main.o \
./src/hs071_nlp.o \
./src/kinematics.o \
./src/polynom.o \
./src/test-pinocchio-fkine.o \
./src/utils.o 

CPP_DEPS += \
./src/generate-randPol-trajectories.d \
./src/hs071_main.d \
./src/hs071_nlp.d \
./src/kinematics.d \
./src/polynom.d \
./src/test-pinocchio-fkine.d \
./src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/python2.7 -I/usr/include/x86_64-linux-gnu/python2.7 -I/usr/local/include/coin-or -O0 -g3 -Wall -c -fmessage-length=0 -DPINOCCHIO_WITH_URDFDOM -DPINOCCHIO_WITH_HPP_FCL -DBOOST_MPL_LIMIT_LIST_SIZE=30 -DHPP_FCL_HAVE_OCTOMAP -DFCL_HAVE_OCTOMAP -DOCTOMAP_MAJOR_VERSION=1 -DOCTOMAP_MINOR_VERSION=9 -DOCTOMAP_PATCH_VERSION=0 -I/opt/openrobots/include -I/usr/local//usr/local/include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


