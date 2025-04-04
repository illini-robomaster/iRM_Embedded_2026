# ---------------------------------------------------------------------- #
#                                                                        #
#  Copyright (C) 2020                                                    #
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

# generic board interface
add_library(board_interface INTERFACE)
target_compile_definitions(board_interface INTERFACE USE_HAL_DRIVER)
target_compile_options(board_interface INTERFACE
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>
    -ffunction-sections -fdata-sections -fmessage-length=0)
target_link_options(board_interface INTERFACE
    -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections
    -Wl,--wrap=malloc -Wl,--wrap=free -Wl,--wrap=calloc -Wl,--wrap=realloc)
target_link_libraries(board_interface INTERFACE c m)

# board specific interfaces
add_subdirectory(DJI_Board_TypeA)
add_subdirectory(DJI_Board_TypeC)
add_subdirectory(DM_MC_01)
