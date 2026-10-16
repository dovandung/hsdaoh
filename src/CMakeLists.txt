# Copyright 2012-2024 Osmocom Project
#
# This file is part of hsdaoh
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

########################################################################
# Setup shared library variant
########################################################################
set(HSDAOH_SOURCES libhsdaoh.c format_convert.c iqconverter_float.c crcspeed.c crc16speed.c crc_simd.c)

if(HSDAOH_BUILD_SHARED)
add_library(hsdaoh SHARED ${HSDAOH_SOURCES})
target_link_libraries(hsdaoh ${LIBUSB_LIBRARIES} ${LIBUVC_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
if(NOT WIN32)
    target_link_libraries(hsdaoh m)
endif()
target_include_directories(hsdaoh PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  # <prefix>/include
  ${LIBUSB_INCLUDE_DIRS}
  ${LIBUVC_INCLUDE_DIRS}
  )
set_target_properties(hsdaoh PROPERTIES DEFINE_SYMBOL "hsdaoh_EXPORTS")
set_target_properties(hsdaoh PROPERTIES OUTPUT_NAME hsdaoh)
set_target_properties(hsdaoh PROPERTIES SOVERSION ${MAJOR_VERSION})
set_target_properties(hsdaoh PROPERTIES VERSION ${LIBVER})
generate_export_header(hsdaoh)
endif()

########################################################################
# Setup static library variant
########################################################################
add_library(hsdaoh_static STATIC ${HSDAOH_SOURCES})
target_link_libraries(hsdaoh_static ${LIBUSB_LIBRARIES} ${LIBUVC_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
if(NOT WIN32)
    target_link_libraries(hsdaoh_static m)
endif()
target_include_directories(hsdaoh_static PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  # <prefix>/include
  ${LIBUSB_INCLUDE_DIRS}
  ${LIBUVC_INCLUDE_DIRS}
  )
set_property(TARGET hsdaoh_static APPEND PROPERTY COMPILE_DEFINITIONS "hsdaoh_STATIC" )
set_property(TARGET hsdaoh_static PROPERTY POSITION_INDEPENDENT_CODE ON)
# Same library filename for static and shared variants (libhsdaoh.a, which
# the pkg-config file's -lhsdaoh finds). On Windows only without the shared
# library, whose import library libhsdaoh.dll.a has that name already.
if(NOT WIN32 OR NOT HSDAOH_BUILD_SHARED)
set_target_properties(hsdaoh_static PROPERTIES OUTPUT_NAME hsdaoh)
endif()
generate_export_header(hsdaoh_static)

########################################################################
# Set up Windows DLL resource files
########################################################################
IF(MSVC)
    include(${CMAKE_SOURCE_DIR}/cmake/Modules/Version.cmake)

    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/hsdaoh.rc.in
        ${CMAKE_CURRENT_BINARY_DIR}/hsdaoh.rc
    @ONLY)
  if(HSDAOH_BUILD_SHARED)
    target_sources(hsdaoh PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/hsdaoh.rc)
  endif()
  target_sources(hsdaoh_static PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/hsdaoh.rc)
ENDIF(MSVC)

########################################################################
# Build utility
########################################################################
if(HSDAOH_BUILD_TOOLS)
if(HSDAOH_BUILD_SHARED)
    set(HSDAOH_TOOL_LIB hsdaoh)
else()
    set(HSDAOH_TOOL_LIB hsdaoh_static)
endif()

add_executable(hsdaoh_file hsdaoh_file.c)
add_executable(hsdaoh_tcp hsdaoh_tcp.c)
add_executable(hsdaoh_test hsdaoh_test.c)
add_executable(hsdaoh_la hsdaoh_la.c)

target_link_libraries(hsdaoh_file ${HSDAOH_TOOL_LIB}
    ${LIBFLAC_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(hsdaoh_tcp ${HSDAOH_TOOL_LIB}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(hsdaoh_test ${HSDAOH_TOOL_LIB}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(hsdaoh_la ${HSDAOH_TOOL_LIB}
    ${CMAKE_THREAD_LIBS_INIT}
)
if(UNIX)
if(APPLE OR CMAKE_SYSTEM MATCHES "OpenBSD")
    target_link_libraries(hsdaoh_test m)
else()
    target_link_libraries(hsdaoh_test m rt)
endif()
endif()

if(WIN32)
target_link_libraries(hsdaoh_tcp ws2_32)
set_property(TARGET hsdaoh_file APPEND PROPERTY COMPILE_DEFINITIONS "hsdaoh_STATIC" )
set_property(TARGET hsdaoh_tcp APPEND PROPERTY COMPILE_DEFINITIONS "hsdaoh_STATIC" )
set_property(TARGET hsdaoh_test APPEND PROPERTY COMPILE_DEFINITIONS "hsdaoh_STATIC" )
set_property(TARGET hsdaoh_la APPEND PROPERTY COMPILE_DEFINITIONS "hsdaoh_STATIC" )
endif()
endif()

########################################################################
# Install built library files & utilities
########################################################################
if(HSDAOH_BUILD_SHARED)
install(TARGETS hsdaoh EXPORT HSDAOH-export
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} # .so/.dylib file
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} # .dll file
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} # .dll.a import library
  )
endif()
install(TARGETS hsdaoh_static EXPORT HSDAOH-export
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} # .a file
  )
if(HSDAOH_BUILD_TOOLS)
install(TARGETS hsdaoh_file hsdaoh_tcp hsdaoh_test hsdaoh_la
  DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()
