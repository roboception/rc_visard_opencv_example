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


find_package(OpenCV REQUIRED COMPONENTS highgui imgproc)
find_package(RC_GENICAM_API REQUIRED)

link_directories(${RC_GENICAM_API_LIBRARY_DIRS})

add_executable(rc_visard_show_streams
  rc_visard_show_streams.cc
  )

target_include_directories(rc_visard_show_streams
  PRIVATE
  ${RC_GENICAM_API_INCLUDE_DIRS}
  )

target_link_libraries(rc_visard_show_streams
  rc_visard_opencv_example
  ${OpenCV_LIBRARIES}
  ${RC_GENICAM_API_LIBRARIES}
  )

install(TARGETS rc_visard_show_streams EXPORT PROJECTTargets COMPONENT bin DESTINATION bin)
