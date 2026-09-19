# -----------------------------------------------------------------------------
# STM32F407VGTx 工程（原 Keil MDK 工程 MDK-ARM/demo.uvprojx）的 CMake 版本
#
# 供 STM32CubeIDE for Visual Studio Code 扩展识别与构建：
#   - 本文件、CMakePresets.json、cmake/target.cmake 三者是扩展识别的依据
#   - 器件/工具链信息在 cmake/target.cmake
#   - 编译/链接选项在 cmake/flags.cmake，源文件清单在 cmake/files.cmake
#   - 本文件由用户维护，可以直接修改（扩展不会重新生成）
# -----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.21)

# 没有通过 CMakePresets.json 配置时（例如手工执行 cmake -B build -G Ninja），
# 也能自动选中 arm-none-eabi 交叉工具链
if(NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake"
      CACHE FILEPATH "交叉编译工具链文件")
endif()

# 生成 compile_commands.json，供 clangd / IntelliSense 索引使用
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 工程名即产物名：demo.elf / demo.hex / demo.bin（与 Keil 输出名一致）
project(demo LANGUAGES C ASM)

# C 语言标准（原 Keil 工程使用 C99）
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

add_executable(${CMAKE_PROJECT_NAME})

# 器件/内核参数、编译链接选项、源文件
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/flags.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/files.cmake)

# -----------------------------------------------------------------------------
# 用户自定义区：新增源文件 / 头文件路径 / 宏 / 链接选项写在这里
# -----------------------------------------------------------------------------
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # 例如：${CMAKE_CURRENT_SOURCE_DIR}/Hardware/新文件.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
)

# -----------------------------------------------------------------------------
# 构建后处理：打印固件大小，并生成 hex / bin（对应 Keil 的 Create HEX File）
# -----------------------------------------------------------------------------
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "生成 ${CMAKE_PROJECT_NAME}.hex / ${CMAKE_PROJECT_NAME}.bin 并输出固件占用"
)
