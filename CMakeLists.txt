cmake_minimum_required(VERSION 3.10)
project(sata C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(sata STATIC
    src/sata_codec.c
    src/sata_phy.c
    src/sata_link.c
    src/sata_fis.c
    src/sata_host.c
    src/sata_device.c)
target_include_directories(sata PUBLIC src)
if(NOT MSVC)
    target_compile_options(sata PRIVATE -Wall -Wextra)
endif()

enable_testing()
add_executable(sata_test test/sata_test.c)
target_link_libraries(sata_test sata)
add_test(NAME sata_test COMMAND sata_test)
