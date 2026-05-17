# Build openblt_flash (LibOpenBLT linked statically).
#
# Linux (from host_openblt_flash/build):
#   cmake .. -G Ninja && cmake --build . --parallel
#   deps: cmake ninja-build build-essential libusb-1.0-0-dev
#
# Windows + MSYS2 UCRT64 (recommended; use build_mingw/ to avoid stale Ninja cache):
#   From firmware/: python scripts/build_host_openblt_flash.py
#   MSYS2 UCRT64 shell: pacman -Syu && pacman -S --needed mingw-w64-ucrt-x86_64-toolchain make cmake mingw-w64-ucrt-x86_64-ninja
#   mkdir -p build_mingw && cd build_mingw && cmake .. -G "MinGW Makefiles" && cmake --build . --parallel
#   Plain PowerShell + gcc often fails until PATH includes /ucrt64/bin (DLLs); use UCRT64 shell or bash -lc with that PATH.

cmake_minimum_required(VERSION 3.15)
project(openblt_flash C)

set(OPENBLT_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../openblt/Host/Source/LibOpenBLT")

macro(collect_include_dirs return_list dir)
  file(GLOB_RECURSE _hdrs "${dir}/*.h")
  set(_dirs "")
  foreach(_f IN LISTS _hdrs)
    get_filename_component(_d "${_f}" DIRECTORY)
    list(APPEND _dirs "${_d}")
  endforeach()
  list(REMOVE_DUPLICATES _dirs)
  set(${return_list} "${_dirs}")
endmacro()

if(WIN32)
  set(PORT_ROOT "${OPENBLT_LIB_DIR}/port/windows")
  if(CMAKE_C_COMPILER_ID MATCHES GNU)
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
      add_compile_definitions(PLATFORM_WINDOWS PLATFORM_32BIT _CRT_SECURE_NO_WARNINGS)
    else()
      add_compile_definitions(PLATFORM_WINDOWS PLATFORM_64BIT _CRT_SECURE_NO_WARNINGS)
    endif()
    set(CMAKE_C_STANDARD 99)
    set(CMAKE_C_EXTENSIONS ON)
  elseif(MSVC)
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
      add_compile_definitions(PLATFORM_WINDOWS PLATFORM_32BIT _CRT_SECURE_NO_WARNINGS)
    else()
      add_compile_definitions(PLATFORM_WINDOWS PLATFORM_64BIT _CRT_SECURE_NO_WARNINGS)
    endif()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
  endif()
elseif(UNIX)
  set(PORT_ROOT "${OPENBLT_LIB_DIR}/port/linux")
  if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    add_compile_definitions(PLATFORM_LINUX PLATFORM_32BIT _TIME_BITS=64 _FILE_OFFSET_BITS=64)
  else()
    add_compile_definitions(PLATFORM_LINUX PLATFORM_64BIT _TIME_BITS=64 _FILE_OFFSET_BITS=64)
  endif()
  set(CMAKE_C_STANDARD 99)
  set(CMAKE_C_EXTENSIONS ON)
endif()

file(GLOB OPENBLT_ROOT_C "${OPENBLT_LIB_DIR}/*.c")
file(GLOB_RECURSE OPENBLT_PORT_C "${PORT_ROOT}/*.c")

collect_include_dirs(PORT_INC_DIRS "${PORT_ROOT}")

add_library(openblt_host STATIC ${OPENBLT_ROOT_C} ${OPENBLT_PORT_C})
target_include_directories(openblt_host PUBLIC "${OPENBLT_LIB_DIR}" ${PORT_INC_DIRS})

add_executable(openblt_flash openblt_flash.c)
target_link_libraries(openblt_flash PRIVATE openblt_host)

if(UNIX)
  target_compile_options(openblt_host PRIVATE -pthread)
  target_link_libraries(openblt_flash PRIVATE pthread usb-1.0 dl)
elseif(WIN32)
  target_link_libraries(openblt_flash PRIVATE ws2_32 winusb setupapi winmm)
endif()
