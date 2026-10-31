# ---------------------------------------------------------------------- #
#                                                                        #
#  Copyright (C) 2025                                                    #
#  Illini RoboMaster @ University of Illinois at Urbana-Champaign.       #
#                                                                        #
#  This program is free software: you can redistribute it and/or modify  #
#  it under the terms of the GNU General Public License as published by   #
#  the Free Software Foundation, either version 3 of the License, or      #
#  (at your option) any later version.                                   #
#                                                                        #
#  This program is distributed in the hope that it will be useful,        #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of         #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#  GNU General Public License for more details.                          #
#                                                                        #
#  You should have received a copy of the GNU General Public License      #
#  along with this program. If not, see <http://www.gnu.org/licenses/>.  #
#                                                                        #
# ---------------------------------------------------------------------- #

# Host (native) regression tests for the DBUS input path. Deliberately a separate
# project: the firmware build configures the ARM cross toolchain for every target,
# and these tests must build with the host compiler and no HAL.
#
#   cmake -S tests/dbus_input -B /tmp/irm-dbus-input-tests
#   cmake --build /tmp/irm-dbus-input-tests
#   (cd /tmp/irm-dbus-input-tests && ctest --output-on-failure)

cmake_minimum_required(VERSION 3.10...3.25)
project(irm_dbus_input_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(test_dbus_input test_dbus_input.cc)
target_include_directories(test_dbus_input PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/libraries
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../vehicles/Omni)
target_compile_options(test_dbus_input PRIVATE -Wall -Wextra -Werror)

find_package(Threads REQUIRED)
target_link_libraries(test_dbus_input PRIVATE Threads::Threads)

add_test(NAME dbus_input COMMAND test_dbus_input)
