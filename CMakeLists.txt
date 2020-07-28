set( CMAKE_VERBOSE_MAKEFILE on )

project(NeowayTest)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(NEOWAY_SDK_DIR $ENV{HOME}/NeowaySDK)
set(NEOWAY_CROSSTOOL_PATH ${NEOWAY_SDK_DIR}/tool/neoway-arm-oe-linux)
set(CMAKE_SYSROOT ${NEOWAY_CROSSTOOL_PATH}/sysroots/armv7a-vfp-neon-oe-linux-gnueabi)
set(CMAKE_STAGING_PREFIX /home/devel/stage)

# SYSROOT.
set(ENV{PATH} ${NEOWAY_CROSSTOOL_PATH}/sysroots/x86_64-oesdk-linux/usr/bin:${NEOWAY_CROSSTOOL_PATH}/sysroots/x86_64-oesdk-linux/usr/bin/arm-oe-linux-gnueabi:$PATH)
set(tools ${NEOWAY_SDK_DIR}/tool/neoway-arm-oe-linux/sysroots/armv7a-vfp-neon-oe-linux-gnueabi)

set(CMAKE_C_COMPILER ${NEOWAY_CROSSTOOL_PATH}/sysroots/x86_64-oesdk-linux/usr/bin/arm-oe-linux-gnueabi-gcc)
set(CMAKE_C_FLAGS -march=armv7-a -marm -mfpu=neon -mfloat-abi=softfp --sysroot=${tools})
set(CMAKE_CXX_COMPILER ${NEOWAY_CROSSTOOL_PATH}/sysroots/x86_64-oesdk-linux/usr/bin/arm-oe-linux-gnueabi-g++)
set(CMAKE_CPP_FLAGS -march=armv7-a -marm -mfpu=neon -mfloat-abi=softfp --sysroot=${tools})
set(CMAKE_MAKE_PROGRAM /usr/bin/make)
set(CMAKE_LD_FLAGS -L/home/skuf1973/NeowaySDK/libs -lnwy_device -lnwy_common -lnwy_service -ldl -pthread)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

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
