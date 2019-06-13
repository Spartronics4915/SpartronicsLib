# You must include me with -DCMAKE_TOOLCHAIN_FILE=roborio_toolchain.cmake
set(Year 2019)

set(ToolchainDir $ENV{HOME}/.gradle/toolchains/frc/${Year}/roborio)
set(ToolchainName arm-frc${Year}-linux-gnueabi)
set(BinaryPrefix ${ToolchainDir}/bin/${ToolchainName})

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_SYSROOT ${ToolchainDir}/${ToolchainName})
set(CMAKE_STAGING_PREFIX ${ToolchainDir}/${ToolchainName})

set(CMAKE_C_COMPILER ${BinaryPrefix}-gcc)
set(CMAKE_CXX_COMPILER ${BinaryPrefix}-g++)
set(CMAKE_AR ${BinaryPrefix}-ar)
set(CMAKE_LINKER ${BinaryPrefix}-ld)
set(CMAKE_OBJCOPY ${BinaryPrefix}-objcopy)
set(CMAKE_OBJDUMP ${BinaryPrefix}-objdump)
set(CMAKE_SIZE ${BinaryPrefix}-size)
set(CMAKE_NM ${BinaryPrefix}-nm)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
