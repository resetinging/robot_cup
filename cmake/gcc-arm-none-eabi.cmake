# GCC 交叉编译工具链（arm-none-eabi-* 需要在 PATH 中，STM32CubeCLT 或
# STM32CubeIDE for VS Code 的 gnu-tools-for-stm32 bundle 都可以提供）

include(${CMAKE_CURRENT_LIST_DIR}/target.cmake)

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# ---------------------------------------------------------------------------
# CPU / FPU / 端序编译参数
# ---------------------------------------------------------------------------
if(CPU STREQUAL "Cortex-M4")
  if(FPU STREQUAL "SP_FPU")
    set(CPU_FLAGS -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
  else()
    set(CPU_FLAGS -mcpu=cortex-m4)
  endif()
elseif(CPU STREQUAL "Cortex-M3")
  set(CPU_FLAGS -mcpu=cortex-m3)
elseif(CPU STREQUAL "Cortex-M7")
  if(FPU STREQUAL "DP_FPU")
    set(CPU_FLAGS -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard)
  elseif(FPU STREQUAL "SP_FPU")
    set(CPU_FLAGS -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard)
  else()
    set(CPU_FLAGS -mcpu=cortex-m7)
  endif()
endif()

if(BYTE_ORDER STREQUAL "Big-endian")
  set(CPU_FLAGS ${CPU_FLAGS} -mbig-endian)
else()
  set(CPU_FLAGS ${CPU_FLAGS} -mlittle-endian)
endif()

set(CPU_FLAGS ${CPU_FLAGS} -mthumb)

# ---------------------------------------------------------------------------
# 工具链程序
# ---------------------------------------------------------------------------
set(TOOLCHAIN_PREFIX   arm-none-eabi-)
set(FLAGS              "-fdata-sections -ffunction-sections -Wno-comment")

set(CMAKE_C_FLAGS      ${FLAGS})
set(CMAKE_ASM_FLAGS    "")

set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE         ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C   ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 各构建类型的差异选项在 cmake/flags.cmake 中显式给出，避免 CMake 默认值干扰
set(CMAKE_C_FLAGS_DEBUG            "")
set(CMAKE_ASM_FLAGS_DEBUG          "")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG   "")
set(CMAKE_C_FLAGS_RELEASE          "")
set(CMAKE_ASM_FLAGS_RELEASE        "")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "")
