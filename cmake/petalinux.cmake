# Source the PetaLinux/Yocto SDK environment-setup script before configuring.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm CACHE STRING "Target CPU (override for a different board)")
if("$ENV{SDKTARGETSYSROOT}" STREQUAL "" OR "$ENV{CXX}" STREQUAL "")
    message(FATAL_ERROR "Source the target SDK environment-setup script (CXX and SDKTARGETSYSROOT required)")
endif()
if(NOT IS_DIRECTORY "$ENV{SDKTARGETSYSROOT}")
    message(FATAL_ERROR "SDKTARGETSYSROOT does not exist")
endif()
set(CMAKE_SYSROOT "$ENV{SDKTARGETSYSROOT}")
set(CMAKE_FIND_ROOT_PATH "${CMAKE_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
# CMake initializes the compiler and flags from the SDK's CXX environment variable.
# Protobuf_PROTOC_EXECUTABLE must point to the matching host protoc, never ARM protoc.
