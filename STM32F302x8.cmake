# Separate toolchain file. for what purposes idk
SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

# Use a custom linker script. Provided in STCube templates
SET(LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/scripts/STM32F303K8Tx_FLASH.ld)
# Linker flags
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections --specs=nosys.specs -T ${LINKER_SCRIPT}")
# -mcpu=cortex-m4 to specify that our MCU has an ARM M4 cpu
SET(COMMON_FLAGS "-mcpu=cortex-m4 -mlittle-endian -mthumb")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -Wall")
