set( CMAKE_VERBOSE_MAKEFILE on )

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_SYSROOT /home/devel/rasp-pi-rootfs)
set(CMAKE_STAGING_PREFIX /home/devel/stage)

# SYSROOT.
set(sdkroot /home/skuf1973/NeowaySDK)
set(tools ${sdkroot}/tool/neoway-arm-oe-linux/sysroots/armv7a-vfp-neon-oe-linux-gnueabi)

set(CMAKE_C_COMPILER {$tools}/arm-oe-linux-gnueabi-gcc -march=armv7-a -marm -mfpu=neon -mfloat-abi=hard --sysroot=${tools})
set(CMAKE_CXX_COMPILER ${tools}/arm-oe-linux-gnueabi-g++ -march=armv7-a -marm -mfpu=neon -mfloat-abi=hard --sysroot=${tools})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

project(NeowayTest)
cmake_minimum_required(VERSION 2.8)

include_directories(
    ${sdkroot}/include
    include
)

link_directories(
    ${sdkroot}/libs
)

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(${PROJECT_NAME}
    nwy_common
    nwy_device
    nwy_service
)
