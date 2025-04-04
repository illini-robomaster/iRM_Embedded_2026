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

include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(COMMON_FLAGS "-Wall -Werror -Wextra -fdiagnostics-color=always")


# default to debug build
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_C_FLAGS "${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -fno-exceptions")

# Debug flags
set(CMAKE_C_FLAGS_DEBUG "-Og -g -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g -DDEBUG")

# Relase flags
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Release with Debug Info flags
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 -g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DDEBUG")
