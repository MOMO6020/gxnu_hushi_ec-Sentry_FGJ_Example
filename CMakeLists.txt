cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
set(CMAKE_PROJECT_NAME gxnu_hushi_ec)
include("cmake/gcc-arm-none-eabi.cmake")
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})
enable_language(C CXX ASM)
add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

add_definitions(
    -DENABLE_RTT_LOG
    -DARM_MATH_CM4
)
file(GLOB_RECURSE APP_SRC
    ${CMAKE_SOURCE_DIR}/app/*.cpp
)
file(GLOB_RECURSE BSP_SRC
    ${CMAKE_SOURCE_DIR}/bsp/*.cpp
    ${CMAKE_SOURCE_DIR}/bsp/*.c
)
file(GLOB_RECURSE MODULE_SRC
    ${CMAKE_SOURCE_DIR}/modules/*.cpp
    ${CMAKE_SOURCE_DIR}/modules/*.c
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${APP_SRC}
    ${BSP_SRC}
    ${MODULE_SRC}
)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)
