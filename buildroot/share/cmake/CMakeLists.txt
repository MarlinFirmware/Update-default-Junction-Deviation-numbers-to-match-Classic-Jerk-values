cmake_minimum_required(VERSION 3.10)
#====================================================================#
# Usage under Linux:                                                 #
#                                                                    #
#    From the project folder:                                        #
#        cd buildroot/share/cmake                                    #
#        mkdir -p build && cd build                                  #
#        cmake ..                                                    #
#        make                                                        #
#                                                                    #
# Usage under Windows:                                               #
#                                                                    #
#    From the project folder:                                        #
#        cd buildroot/share/cmake                                    #
#        mkdir build && cd build                                     #
#        cmake -G"Unix Makefiles" ..                                 #
#        make                                                        #
#====================================================================#

#====================================================================#
# Download marlin-cmake scriptfiles if not already installed         #
# and add the path to the module path                                #
#====================================================================#

set(SCRIPT_BRANCH 1.0.2) # Set to wanted marlin-cmake release tag or branch

if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake)

  file(DOWNLOAD https://github.com/tohara/marlin-cmake/archive/${SCRIPT_BRANCH}.tar.gz
                ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake-src.tar.gz SHOW_PROGRESS)

  execute_process(COMMAND ${CMAKE_COMMAND} -E tar -xvf ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake-src.tar.gz WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

  file(RENAME ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake-${SCRIPT_BRANCH} ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake)
  file(REMOVE ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake-src.tar.gz)

endif()

if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/modules/Arduino_SDK.cmake)
  file(DOWNLOAD https://raw.githubusercontent.com/tohara/marlin-cmake/master/modules/Arduino_SDK.cmake
                ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/modules/Arduino_SDK.cmake SHOW_PROGRESS)
endif()

if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/modules/marlin_cmake_functions.cmake)
  file(DOWNLOAD https://raw.githubusercontent.com/tohara/marlin-cmake/master/modules/marlin_cmake_functions.cmake
                ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/modules/marlin_cmake_functions.cmake SHOW_PROGRESS)
endif()

if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/Platform/Arduino.cmake)
  file(DOWNLOAD https://raw.githubusercontent.com/tohara/marlin-cmake/master/Platform/Arduino.cmake
                ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/Platform/Arduino.cmake SHOW_PROGRESS)
endif()

if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/settings/marlin_boards.txt)
  file(DOWNLOAD https://raw.githubusercontent.com/tohara/marlin-cmake/master/settings/marlin_boards.txt
                ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/settings/marlin_boards.txt SHOW_PROGRESS)
endif()

if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/toolchain/ArduinoToolchain.cmake)
  file(DOWNLOAD https://raw.githubusercontent.com/tohara/marlin-cmake/master/toolchain/ArduinoToolchain.cmake
                ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/toolchain/ArduinoToolchain.cmake SHOW_PROGRESS)
endif()

if(WIN32)
  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/resources/make.exe)
    file(DOWNLOAD https://raw.githubusercontent.com/tohara/marlin-cmake/master/resources/make.exe
                  ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/resources/make.exe SHOW_PROGRESS)
  endif()
endif(WIN32)

if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/arduino-1.8.19)

  file(DOWNLOAD https://downloads.arduino.cc/arduino-1.8.19-windows.zip
                ${CMAKE_CURRENT_LIST_DIR}/arduino-1.8.19-windows.zip SHOW_PROGRESS)

  execute_process(COMMAND ${CMAKE_COMMAND} -E tar -xvzf ${CMAKE_CURRENT_LIST_DIR}/arduino-1.8.19-windows.zip WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

  file(REMOVE ${CMAKE_CURRENT_LIST_DIR}/arduino-1.8.19-windows.zip)

endif()

# Print CMake version
message("-- Running CMake version: " ${CMAKE_VERSION})

# Replace the CMake Ver. in the Arduino.cmake
file(READ "${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/Platform/Arduino.cmake" ORIGINAL_FILE_CONTENTS)
string(REGEX REPLACE "cmake_minimum_required\\(VERSION[^\n]*\n" "cmake_minimum_required(VERSION 3.10)\n" NEW_FILE_CONTENTS "${ORIGINAL_FILE_CONTENTS}")
file(WRITE "${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/Platform/Arduino.cmake" "${NEW_FILE_CONTENTS}")

# Fix "CMake Error... avr-gcc / avr-g++ is not a full path..."
if(UNIX)
  set(ORIGINAL_FILE_CONTENTS "")
  set(NEW_FILE_CONTENTS "")
  file(READ "${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/toolchain/ArduinoToolchain.cmake" ORIGINAL_FILE_CONTENTS)
  string(REPLACE "set(CMAKE_C_COMPILER   avr-gcc)\nset(CMAKE_CXX_COMPILER avr-g++)"
                 "set(CMAKE_C_COMPILER   /usr/bin/gcc)\nset(CMAKE_CXX_COMPILER /usr/bin/gcc)"
                 NEW_FILE_CONTENTS "${ORIGINAL_FILE_CONTENTS}")
  file(WRITE "${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/toolchain/ArduinoToolchain.cmake" "${NEW_FILE_CONTENTS}")
endif(UNIX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/modules)

#====================================================================#
#  Custom path to Arduino SDK can be set here                        #
#  It can also be set from command line. eg.:                        #
#  cmake .. -DARDUINO_SDK_PATH="/path/to/arduino-1.x.x"              #
#====================================================================#
set(ARDUINO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/arduino-1.8.19)
#set(ARDUINO_SDK_PATH /Applications/Arduino.app/Contents/Java)
#set(ARDUINO_SDK_PATH $HOME/ArduinoAddons/Arduino_1.6.x)

#====================================================================#
#  Set included cmake files                                          #
#====================================================================#
include(Arduino_SDK) # Find the intallpath of Arduino SDK
include(marlin_cmake_functions)

#====================================================================#
#  Set toolchain file for arduino                                    #
#====================================================================#
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/marlin-cmake/toolchain/ArduinoToolchain.cmake) # Arduino Toolchain

#====================================================================#
#  Setup Project                                                     #
#                                                                    #
#  If you receive this error:                                        #
#   'Unknown CMake command "_cmake_record_install_prefix".'          #
#                                                                    #
#   Go to the file in your CMake directory                           #
#                                                                    #
#   For Windows: cmake\Modules\Platform\WindowsPaths.cmake           #
#   For macOS: cmake/Modules/Platform/UnixPaths.cmake                #
#   For Linux: cmake/Modules/Platform/UnixPaths.cmake                #
#                                                                    #
#   Comment out "_cmake_record_install_prefix()"                     #
#     - OR -                                                         #
#   Add "include(CMakeSystemSpecificInformation)" above the line     #
#                                                                    #
#====================================================================#
project(Marlin C CXX)

#====================================================================#
#  Register non standard hardware                                    #
#====================================================================#
#register_hardware_platform(/home/tom/test/Sanguino)

#====================================================================#
#  Print any info                                                    #
#  print_board_list()                                                #
#  print_programmer_list()                                           #
#  print_board_settings(mega)                                        #
#====================================================================#
print_board_list()
print_programmer_list()

#====================================================================#
#  Get motherboard settings from Configuration.h                     #
#  setup_motherboard(TARGET Marlin_src_folder)                       #
#  Returns ${TARGET}_BOARD and ${TARGET}_CPU                         #
#                                                                    #
#  To set it manually:                                               #
#    set(${PROJECT_NAME}_BOARD mega)                                 #
#    set(${PROJECT_NAME}_CPU atmega2560)                             #
#====================================================================#
setup_motherboard(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/../../../Marlin)

#====================================================================#
#  Setup all source files                                            #
#  Include Marlin.ino to compile libs not included in *.cpp files    #
#====================================================================#

file(GLOB_RECURSE SOURCES "../../../Marlin/*.cpp")
set(${PROJECT_NAME}_SRCS "${SOURCES};../../../Marlin/Marlin.ino")

#====================================================================#
#  Define the port for uploading code to the Arduino                 #
#  Can be set from commandline with:                                 #
#    cmake .. -DUPLOAD_PORT=/dev/ttyACM0                             #
#====================================================================#
if(UPLOAD_PORT)
  set(${PROJECT_NAME}_PORT ${UPLOAD_PORT})
else()
  set(${PROJECT_NAME}_PORT /dev/ttyACM0)
endif()

#====================================================================#
#  Register arduino libraries not included in SDK                    #
#====================================================================#
#link_directories(/home/tom/test/ArduinoAddons) #U8glib
#set(${PROJECT_NAME}_ARDLIBS U8glib)
#set(U8glib_RECURSE True)

#====================================================================#
# Command to generate code arduino firmware (.hex file)              #
#====================================================================#
generate_arduino_firmware(${PROJECT_NAME})
