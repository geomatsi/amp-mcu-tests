INCLUDE(CMakeForceCompiler)

CMAKE_MINIMUM_REQUIRED (VERSION 3.13)

project(imx8qxp-rpmsg-can)

# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)
 
# SELECT BOARD

set(SUPPORTED_BOARDS digi8x icore8x)

IF(NOT BOARD)
	MESSAGE(FATAL_ERROR "*** Please set BOARD ***")
ENDIF()

IF(NOT BOARD IN_LIST SUPPORTED_BOARDS)
	message(FATAL_ERROR "BOARD must be one of ${SUPPORTED_BOARDS}")
ENDIF()

# MCUEXPRESSO DIRECTORY
IF(NOT MCUXPRESSO_PATH)
	MESSAGE(FATAL_ERROR "***Please set MCUXPRESSO_PATH ***")
ENDIF()

# CURRENT PROJECT DIRECTORY
SET(PROJECT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# FIRMWARE
add_executable(freertos_rpmsg_can.elf "")

#
# RELEASE FLAGS
#

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -DNDEBUG")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -D__STARTUP_CLEAR_BSS")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -D__STARTUP_INITIALIZE_NONCACHEDATA")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mcpu=cortex-m4")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -Wall")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -Wno-address-of-packed-member")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mfloat-abi=hard")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mfpu=fpv4-sp-d16")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mthumb")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -fno-common")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -ffunction-sections")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -fdata-sections")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -ffreestanding")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -fno-builtin")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mapcs")
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -std=gnu99")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DNDEBUG")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DCPU_MIMX8QX6AVLFZ")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DBOARD_USE_SCFW_IRQ")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DSDK_I2C_BASED_COMPONENT_USED=1")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DCODEC_CS42888_ENABLE")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DSERIAL_PORT_TYPE_UART=1")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DFSL_RTOS_FREE_RTOS")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Os")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mcpu=cortex-m4")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wno-address-of-packed-member")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mfloat-abi=hard")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mfpu=fpv4-sp-d16")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mthumb")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -MMD")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -MP")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fno-strict-aliasing")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fno-common")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -ffunction-sections")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fdata-sections")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -ffreestanding")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fno-builtin")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mapcs")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -std=gnu99")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DSERIAL_PORT_TYPE_UART=1")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Os")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mcpu=cortex-m4")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wno-address-of-packed-member")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mfloat-abi=hard")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mfpu=fpv4-sp-d16")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mthumb")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -MMD")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -MP")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-common")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ffunction-sections")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fdata-sections")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ffreestanding")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-builtin")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mapcs")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-rtti")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-exceptions")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mcpu=cortex-m4")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Wall")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mfloat-abi=hard")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mfpu=fpv4-sp-d16")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --specs=nano.specs")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --specs=nosys.specs")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -fno-common")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -ffunction-sections")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -fdata-sections")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -ffreestanding")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -fno-builtin")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mthumb")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mapcs")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --gc-sections")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -static")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -z")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} muldefs")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Map=output.map")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --defsym=__stack_size__=0x400")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --defsym=__heap_size__=0x400")


#
# DEBUG FLAGS
#

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -DDEBUG")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -D__STARTUP_CLEAR_BSS")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -D__STARTUP_INITIALIZE_NONCACHEDATA")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -g")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mcpu=cortex-m4")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -Wall")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -Wno-address-of-packed-member")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mfloat-abi=hard")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mfpu=fpv4-sp-d16")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mthumb")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -fno-common")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -ffunction-sections")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -fdata-sections")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -ffreestanding")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -fno-builtin")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mapcs")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -std=gnu99")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DCPU_MIMX8QX6AVLFZ")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DBOARD_USE_SCFW_IRQ")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DSDK_I2C_BASED_COMPONENT_USED=1")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DCODEC_CS42888_ENABLE")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DSERIAL_PORT_TYPE_UART=1")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DFSL_RTOS_FREE_RTOS")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mcpu=cortex-m4")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wno-address-of-packed-member")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mfloat-abi=hard")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mfpu=fpv4-sp-d16")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mthumb")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -MMD")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -MP")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-strict-aliasing")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-common")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -ffunction-sections")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fdata-sections")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -ffreestanding")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-builtin")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mapcs")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -std=gnu99")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DSERIAL_PORT_TYPE_UART=1")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mcpu=cortex-m4")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wno-address-of-packed-member")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mfloat-abi=hard")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mfpu=fpv4-sp-d16")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mthumb")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -MMD")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -MP")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-common")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ffunction-sections")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fdata-sections")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ffreestanding")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-builtin")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mapcs")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-rtti")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-exceptions")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mcpu=cortex-m4")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Wall")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mfloat-abi=hard")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mfpu=fpv4-sp-d16")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --specs=nano.specs")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --specs=nosys.specs")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fno-common")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -ffunction-sections")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fdata-sections")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -ffreestanding")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fno-builtin")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mthumb")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mapcs")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --gc-sections")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -static")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -z")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} muldefs")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Map=output.map")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --defsym=__stack_size__=0x400")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --defsym=__heap_size__=0x400")

#
# MCUXPRESSO FILES
#

include_directories(${MCUXPRESSO_PATH}/CMSIS/Include)
include_directories(${MCUXPRESSO_PATH}/components/codec/cs42888)
include_directories(${MCUXPRESSO_PATH}/components/codec)
include_directories(${MCUXPRESSO_PATH}/components/codec/port/cs42888)
include_directories(${MCUXPRESSO_PATH}/components/codec/i2c)
include_directories(${MCUXPRESSO_PATH}/components/i2c)
include_directories(${MCUXPRESSO_PATH}/components/uart)
include_directories(${MCUXPRESSO_PATH}/components/serial_manager)
include_directories(${MCUXPRESSO_PATH}/components/lists)
include_directories(${MCUXPRESSO_PATH}/devices)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/main)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/irq)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/misc)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/pad)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/pm)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/rm)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/seco)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/timer)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/utilities)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/utilities/str)
include_directories(${MCUXPRESSO_PATH}/devices/MIMX8QX6/utilities/debug_console)
include_directories(${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/include/platform/imx8qx_cm4)
include_directories(${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/include)
include_directories(${MCUXPRESSO_PATH}/middleware/multicore/remoteproc)
include_directories(${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include)
include_directories(${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/portable/GCC/ARM_CM4F)

target_sources(freertos_rpmsg_can.elf
	PRIVATE
		"${MCUXPRESSO_PATH}/CMSIS/Include/core_cm4.h"
		"${MCUXPRESSO_PATH}/CMSIS/Include/mpu_armv7.h"
		"${MCUXPRESSO_PATH}/CMSIS/Include/cmsis_armcc.h"
		"${MCUXPRESSO_PATH}/CMSIS/Include/cmsis_armclang.h"
		"${MCUXPRESSO_PATH}/CMSIS/Include/cmsis_armclang_ltm.h"
		"${MCUXPRESSO_PATH}/CMSIS/Include/cmsis_compiler.h"
		"${MCUXPRESSO_PATH}/CMSIS/Include/cmsis_gcc.h"
		"${MCUXPRESSO_PATH}/CMSIS/Include/cmsis_iccarm.h"
		"${MCUXPRESSO_PATH}/CMSIS/Include/cmsis_version.h"
		"${MCUXPRESSO_PATH}/CMSIS/Include/arm_common_tables.h"
		"${MCUXPRESSO_PATH}/CMSIS/Include/arm_const_structs.h"
		"${MCUXPRESSO_PATH}/CMSIS/Include/arm_math.h"
		"${MCUXPRESSO_PATH}/components/codec/cs42888/fsl_cs42888.c"
		"${MCUXPRESSO_PATH}/components/codec/cs42888/fsl_cs42888.h"
		"${MCUXPRESSO_PATH}/components/codec/fsl_codec_common.c"
		"${MCUXPRESSO_PATH}/components/codec/fsl_codec_common.h"
		"${MCUXPRESSO_PATH}/components/codec/port/cs42888/fsl_codec_adapter.c"
		"${MCUXPRESSO_PATH}/components/codec/port/cs42888/fsl_codec_adapter.h"
		"${MCUXPRESSO_PATH}/components/codec/i2c/fsl_codec_i2c.c"
		"${MCUXPRESSO_PATH}/components/codec/i2c/fsl_codec_i2c.h"
		"${MCUXPRESSO_PATH}/components/i2c/i2c.h"
		"${MCUXPRESSO_PATH}/components/i2c/lpi2c_adapter.c"
		"${MCUXPRESSO_PATH}/components/uart/lpuart_adapter.c"
		"${MCUXPRESSO_PATH}/components/uart/uart.h"
		"${MCUXPRESSO_PATH}/components/serial_manager/serial_manager.c"
		"${MCUXPRESSO_PATH}/components/serial_manager/serial_manager.h"
		"${MCUXPRESSO_PATH}/components/serial_manager/serial_port_internal.h"
		"${MCUXPRESSO_PATH}/components/lists/generic_list.c"
		"${MCUXPRESSO_PATH}/components/lists/generic_list.h"
		"${MCUXPRESSO_PATH}/components/serial_manager/serial_port_uart.c"
		"${MCUXPRESSO_PATH}/components/serial_manager/serial_port_uart.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/MIMX8QX6_cm4.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/MIMX8QX6_cm4_features.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_irqsteer.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_irqsteer.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_lpi2c.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_lpi2c.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_common.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_common.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_clock.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_clock.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_flexcan.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_flexcan.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_mu.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_mu.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_gpio.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_gpio.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_edma.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_edma.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_memory.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_lpi2c_freertos.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_lpi2c_freertos.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_sai_edma.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_sai_edma.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_sai.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_sai.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_lpuart.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/drivers/fsl_lpuart.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/fsl_device_registers.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/gcc/startup_MIMX8QX6_cm4.S"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/main/imx8qx_pads.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/main/ipc.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/main/ipc_imx8qx.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/main/rpc.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/main/scfw.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/main/types.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/irq/irq_api.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/irq/irq_rpc.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/irq/irq_rpc_clnt.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/misc/misc_api.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/misc/misc_rpc.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/misc/misc_rpc_clnt.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/pad/pad_api.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/pad/pad_rpc.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/pad/pad_rpc_clnt.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/pm/pm_api.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/pm/pm_rpc.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/pm/pm_rpc_clnt.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/rm/rm_api.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/rm/rm_rpc.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/rm/rm_rpc_clnt.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/seco/seco_api.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/seco/seco_rpc.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/seco/seco_rpc_clnt.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/timer/timer_api.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/timer/timer_rpc.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/scfw_api/svc/timer/timer_rpc_clnt.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/system_MIMX8QX6_cm4.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/system_MIMX8QX6_cm4.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/utilities/debug_console/fsl_debug_console.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/utilities/debug_console/fsl_debug_console.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/utilities/debug_console/fsl_debug_console_conf.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/utilities/str/fsl_str.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/utilities/str/fsl_str.h"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/utilities/fsl_assert.c"
		"${MCUXPRESSO_PATH}/devices/MIMX8QX6/utilities/fsl_sbrk.c"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/include/platform/imx8qx_cm4/rpmsg_platform.h"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/rpmsg_lite/porting/platform/imx8qx_cm4/rpmsg_platform.c"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/include/rpmsg_env.h"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/include/rpmsg_queue.h"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/rpmsg_lite/porting/environment/rpmsg_env_freertos.c"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/rpmsg_lite/rpmsg_queue.c"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/common/llist.c"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/include/llist.h"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/include/rpmsg_compiler.h"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/include/rpmsg_default_config.h"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/include/rpmsg_lite.h"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/include/rpmsg_ns.h"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/include/virtio_ring.h"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/include/virtqueue.h"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/rpmsg_lite/rpmsg_lite.c"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/rpmsg_lite/rpmsg_ns.c"
		"${MCUXPRESSO_PATH}/middleware/multicore/rpmsg_lite/lib/virtio/virtqueue.c"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/croutine.c"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/event_groups.c"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/queue.c"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/list.c"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/stream_buffer.c"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/tasks.c"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/timers.c"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/FreeRTOS.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/StackMacros.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/atomic.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/croutine.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/deprecated_definitions.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/event_groups.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/freertos_tasks_c_additions.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/list.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/message_buffer.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/mpu_prototypes.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/mpu_wrappers.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/portable.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/projdefs.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/queue.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/semphr.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/stack_macros.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/stream_buffer.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/task.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/include/timers.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/portable/GCC/ARM_CM4F/port.c"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/portable/GCC/ARM_CM4F/portmacro.h"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/portable/MemMang/ReadMe.url"
		"${MCUXPRESSO_PATH}/rtos/freertos/freertos_kernel/portable/MemMang/heap_4.c"
)

#
# PROJECT FILES
#

include_directories(${PROJECT_PATH})
include_directories(${PROJECT_PATH}/platform/common)

target_sources(freertos_rpmsg_can.elf
	PRIVATE
		"${PROJECT_PATH}/platform/common/clock_config.c"
		"${PROJECT_PATH}/platform/common/clock_config.h"
		"${PROJECT_PATH}/platform/common/board.c"
		"${PROJECT_PATH}/platform/common/board.h"
		"${PROJECT_PATH}/platform/${BOARD}/pin_mux.c"
		"${PROJECT_PATH}/platform/${BOARD}/pin_mux.h"
		"${PROJECT_PATH}/platform/${BOARD}/hw.c"
		"${PROJECT_PATH}/main.c"
		"${PROJECT_PATH}/FreeRTOSConfig.h"
		"${PROJECT_PATH}/rpmsg_config.h"
		"${PROJECT_PATH}/can-rpmsg-ipc.h"
		"${PROJECT_PATH}/remoteproc.h"
		"${PROJECT_PATH}/rsc_table.c"
		"${PROJECT_PATH}/rsc_table.h"
		"${PROJECT_PATH}/compat_linux.h"
		"${PROJECT_PATH}/compat_linux.c"
)

set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -T${PROJECT_PATH}/MIMX8QX6xxxxx_cm4_ram.ld -static")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${PROJECT_PATH}/MIMX8QX6xxxxx_cm4_ram.ld -static")

target_link_libraries(freertos_rpmsg_can.elf -Wl,--start-group)
target_link_libraries(freertos_rpmsg_can.elf debug m)
target_link_libraries(freertos_rpmsg_can.elf debug c)
target_link_libraries(freertos_rpmsg_can.elf debug gcc)
target_link_libraries(freertos_rpmsg_can.elf debug nosys)
target_link_libraries(freertos_rpmsg_can.elf optimized m)
target_link_libraries(freertos_rpmsg_can.elf optimized c)
target_link_libraries(freertos_rpmsg_can.elf optimized gcc)
target_link_libraries(freertos_rpmsg_can.elf optimized nosys)
target_link_libraries(freertos_rpmsg_can.elf optimized m)
target_link_libraries(freertos_rpmsg_can.elf optimized c)
target_link_libraries(freertos_rpmsg_can.elf optimized gcc)
target_link_libraries(freertos_rpmsg_can.elf optimized nosys)
target_link_libraries(freertos_rpmsg_can.elf optimized m)
target_link_libraries(freertos_rpmsg_can.elf optimized c)
target_link_libraries(freertos_rpmsg_can.elf optimized gcc)
target_link_libraries(freertos_rpmsg_can.elf optimized nosys)
target_link_libraries(freertos_rpmsg_can.elf optimized m)
target_link_libraries(freertos_rpmsg_can.elf optimized c)
target_link_libraries(freertos_rpmsg_can.elf optimized gcc)
target_link_libraries(freertos_rpmsg_can.elf optimized nosys)
target_link_libraries(freertos_rpmsg_can.elf optimized m)
target_link_libraries(freertos_rpmsg_can.elf optimized c)
target_link_libraries(freertos_rpmsg_can.elf optimized gcc)
target_link_libraries(freertos_rpmsg_can.elf optimized nosys)
target_link_libraries(freertos_rpmsg_can.elf -Wl,--end-group)

ADD_CUSTOM_COMMAND(TARGET freertos_rpmsg_can.elf POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Obinary ${EXECUTABLE_OUTPUT_PATH}/freertos_rpmsg_can.elf ${EXECUTABLE_OUTPUT_PATH}/freertos_rpmsg_can.bin)
