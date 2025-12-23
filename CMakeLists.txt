# ---------------------------------------------------------------------- #
#                                                                        #
#  Copyright (C) 2022                                                    #
#  Illini RoboMaster @ University of Illinois at Urbana-Champaign.       #
#                                                                        #
#  This program is free software: you can redistribute it and/or modify  #
#  it under the terms of the GNU General Public License as published by  #
#  the Free Software Foundation, either version 3 of the License, or     #
#  (at your option) any later version.                                   #
#                                                                        #
#  This program is distributed in the hope that it will be useful,       #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#  GNU General Public License for more details.                          #
#                                                                        #
#  You should have received a copy of the GNU General Public License     #
#  along with this program. If not, see <http://www.gnu.org/licenses/>.  #
#                                                                        #
# ---------------------------------------------------------------------- #

cmake_minimum_required(VERSION 3.10)

include(cmake/arm_toolchain.cmake)
include(cmake/build_helper.cmake)
include(cmake/clang_format.cmake)
include(cmake/doxygen.cmake)

project(iRM_Embedded_2023)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

add_subdirectory(boards)
add_subdirectory(shared)
add_subdirectory(vehicles)
add_subdirectory(examples)
