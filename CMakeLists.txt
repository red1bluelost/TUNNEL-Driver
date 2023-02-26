cmake_minimum_required(VERSION 3.18)
project(tunnel-driver VERSION 0.0.0)

set(CMAKE_VERBOSE_MAKEFILE
    ON
    CACHE BOOL "THIS IS TEMPORARY" FORCE)
add_compile_options(-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-D__KERNEL__ -DMODULE -DKBUILD_MODNAME="whatever"
                -DPAGE_SIZE=4096)

set(KERNEL_DIR
    ""
    CACHE STRING "Path to the kernel build directory")
if ("${KERNEL_DIR}" STREQUAL "")
  execute_process(
    COMMAND uname -r
    OUTPUT_VARIABLE UNAME_R
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(KERNEL_DIR "/lib/modules/${UNAME_R}/build")
endif ()
find_file(
  KERNEL_MAKEFILE
  NAMES Makefile
  PATHS ${KERNEL_DIR}
  NO_DEFAULT_PATH)
if (NOT KERNEL_MAKEFILE)
  message(FATAL_ERROR "There is no Makefile in KERNEL_DIR! ${KERNEL_DIR}")
endif ()

include_directories(SYSTEM ${KERNEL_DIR}/include ${KERNEL_DIR}/arch/x86/include
                    ${KERNEL_DIR}/arch/x86/include/generated)

add_library(tunnel_network_lib STATIC tunnel_usb.c tunnel_usb.h)
get_target_property(MODULE_SRCS tunnel_network_lib SOURCES)

set(DRIVER_NAME tunnel)
set(DRIVER_MAIN tunnel_network.c)
set(DRIVER_KOBJ ${DRIVER_NAME}.ko)

list(APPEND MODULE_SRCS ${DRIVER_MAIN})

set(MODULE_SRCS_STR ${MODULE_SRCS})
list(FILTER MODULE_SRCS_STR INCLUDE REGEX ".c")
string(REPLACE ";" " " MODULE_SRCS_STR "${MODULE_SRCS_STR}")
string(REPLACE ".c" ".o" MODULE_SRCS_STR "${MODULE_SRCS_STR}")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Kbuild.in
               ${CMAKE_CURRENT_SOURCE_DIR}/Kbuild @ONLY)

foreach(src ${MODULE_SRCS})
  configure_file(${src} ${src} COPYONLY)
endforeach()

set(KBUILD_CMD ${CMAKE_MAKE_PROGRAM} -C ${KERNEL_DIR} modules
               M=${CMAKE_CURRENT_BINARY_DIR} src=${CMAKE_CURRENT_SOURCE_DIR})
add_custom_command(
  OUTPUT ${DRIVER_KOBJ}
  COMMAND ${KBUILD_CMD}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS ${MODULE_SRCS}
  VERBATIM)

add_custom_target(${DRIVER_NAME} ALL DEPENDS ${DRIVER_KOBJ})
add_custom_target(${DRIVER_NAME}-clean COMMAND ${CMAKE_MAKE_PROGRAM} clean)

add_library(dummy_target ${MODULE_SRCS} ${DRIVER_MAIN})
