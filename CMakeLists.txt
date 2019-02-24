# Project needs to support both C and Assembly (ASM)
project(arm-examples C ASM)
cmake_minimum_required(VERSION 3.12)

# MCU flag to tell the STCube library what MCU it's dealing with
add_definitions(-DSTM32F302x8)

# Link necessary MCU support files to compiler
include_directories(${CMAKE_SOURCE_DIR}/library)

# Should compile all user .c code
file(GLOB_RECURSE PROGRAM_SOURCES "code/*.c")
# We need to add STM startup code to the final .hex and unless we want to make that ourselves we need to add this file
file(GLOB_RECURSE STARTUP_SFILE "library/startup_stm32f302x8.s")

# Compile all the code into a .elf file
add_executable(${PROJECT_NAME}.elf ${PROGRAM_SOURCES} ${STARTUP_SFILE} ${LINKER_SCRIPT})

# Convert the .elf into .hex so it can be burnt onto the MCU, and copy it into the build/ folder
set(HEX_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.hex)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
		COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
		COMMENT "Building ${HEX_FILE}"
		COMMAND ${PROJECT_SOURCE_DIR}/flash-stm32f302r8
		COMMENT "Flashing to STM32f302r8"
		)
