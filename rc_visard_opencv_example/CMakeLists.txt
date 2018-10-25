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

find_package(OpenCV REQUIRED COMPONENTS core)
find_package(RC_GENICAM_API 2.0 REQUIRED)

message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

link_directories(${RC_GENICAM_API_LIBRARY_DIRS})

file(GLOB_RECURSE SOURCES "*.cc")
file(GLOB_RECURSE HEADERS "*.h")
file(GLOB_RECURSE PUBLIC_HEADERS "../include/rc_visard_opencv_example/*.h")

set(LIBRARY_NAME ${PROJECT_NAME})

add_library(${LIBRARY_NAME} SHARED
  ${SOURCES}
  ${HEADERS}
  ${PUBLIC_HEADERS}
  ${CMAKE_CURRENT_BINARY_DIR}/project_version.cc
  )

target_include_directories(${LIBRARY_NAME}
  PRIVATE
  ${RC_GENICAM_API_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${LIBRARY_NAME}
  PRIVATE
  ${RC_GENICAM_API_LIBRARIES}
  PUBLIC
  ${OpenCV_LIBRARIES}
  )

install(TARGETS ${LIBRARY_NAME} EXPORT PROJECTTargets COMPONENT bin
		RUNTIME DESTINATION bin
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib)

install(FILES ${PUBLIC_HEADERS} COMPONENT dev DESTINATION include/${PROJECT_NAME})
