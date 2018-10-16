# This file is part of the rc_visard_opencv_example package.
#
# Copyright (c) 2018 Roboception GmbH
# All rights reserved
#
# Author: Raphael Schaller
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required (VERSION 2.8.12)

# Don't set PROJECT_VERSION to empty string when no VERSION is given to project() command.
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 OLD)
endif()

project(rc_visard_opencv_example)
include(cmake/project_version.cmake)

enable_testing()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# - Configuration for code optimization -

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RELEASE CACHE STRING "Build type: DEBUG or RELEASE" FORCE)
endif ()

# - Standard definitions -

add_definitions(-std=c++11)
add_definitions(-Wall)

# In case of Visual Studio, enable exporting all symbols and disable some
# warnings

if (WIN32)
  add_definitions(/DGENICAM_NO_AUTO_IMPLIB)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif ()

if (MSVC)
  add_definitions("/wd4003")
  add_definitions("/wd4514")
  add_definitions("/wd4710")
  add_definitions("/wd4820")
  add_definitions("/wd4435")
  add_definitions("/wd4668")
  add_definitions("/wd4127")
  add_definitions("/wd4265")
  add_definitions("/wd4996")
  add_definitions("/wd4437")
  add_definitions("/wd4571")
  add_definitions("/wd4355")
  add_definitions("/wd4061")
  add_definitions("/wd4800")
  add_definitions("/wd4711")
endif ()

# - Build individual parts -

add_subdirectory(rc_visard_opencv_example)
add_subdirectory(tools)

# export project targets

# ??? Declare (shared) libraries that are created by your project. For all
# libraries that appear here, the following line should exist in a cmake
# file in a sub directory:
# install(TARGETS rcXXX EXPORT PROJECTTargets COMPONENT bin DESTINATION lib)
set(PROJECT_LIBRARIES ${PROJECT_NAME})

# ??? Declare static libraries that are created by your project. For all
# libraries that appear here, the following line should exist in a cmake
# file in a sub directory:
# install(TARGETS rcXXX_static EXPORT PROJECTTargets COMPONENT dev DESTINATION lib)
# set(PROJECT_STATIC_LIBRARIES rcXXX_static)

# ??? install generated version.h file
# the destination path might have to be adjusted for your project
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/project_version.h COMPONENT dev DESTINATION include/${PROJECT_NAME})

include(cmake/configure_link_libs.cmake)

# add a target to generate API documentation with Doxygen

add_subdirectory(doc)

# - Define information for packaging -

# optionally set description and maintainer (if not set will use defaults from package_debian.cmake)
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Roboception rc_visard OpenCV example")

# optionally set package maintainer (otherwise defaults to Roboception <info@roboception.de>)
#set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Your Name <your.name@roboception.de>")

# optionally specify dependencies of the debian package here (comma separated!)
#set(CPACK_DEBIAN_PACKAGE_DEPENDS "cvkit (>= 1.7.0)")

set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF)

add_custom_target(gen_deps)

add_custom_command(TARGET gen_deps POST_BUILD
  COMMAND ${CMAKE_COMMAND}
  ARGS -D BINARY_DIR=${CMAKE_BINARY_DIR}
  -P ${CMAKE_SOURCE_DIR}/cmake/add_target_dependencies.cmake
  COMMENT "Generating dependency list" VERBATIM)

include(cmake/package_debian.cmake)
