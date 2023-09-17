cmake_minimum_required(VERSION 3.16)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
set(CMAKE_C_COMPILER arm-none-eabi-gcc CACHE STRING "C Compiler")
set(CMAKE_CXX_COMPILER arm-none-eabi-g++ CACHE STRING "C++ Compiler")

project(TripSitter C CXX ASM)

add_executable(main.elf
  system/vector_table.S
  system/reset_handler.c
)

add_subdirectory(src)

target_include_directories(main.elf PUBLIC include)

target_compile_options(main.elf PUBLIC
  -Wall
  -nostdlib
  -march=armv7+nofp
  -ffunction-sections
  -fdata-sections
  -mthumb
  -g
)

target_link_options(main.elf PUBLIC
  --specs=nosys.specs
  -nostdlib
  -T${CMAKE_SOURCE_DIR}/system/linker.ld
  -Wl,-Map=linker.map
)

add_custom_target(debug
  DEPENDS main.elf
  COMMAND arm-none-eabi-gdb -q -tui
  -ex "target extended-remote localhost:3333"
  -ex "set confirm off" -ex "file main.elf" -ex "load"
  -ex "tb main" -ex "run" -ex "set confirm on"
)

add_custom_target(flash
  DEPENDS main.elf
  COMMAND openocd -f "interface/jlink.cfg" -c "transport select swd"
  -f "target/efm32.cfg" -c "program main.elf verify reset exit"
)