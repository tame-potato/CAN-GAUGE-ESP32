# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/opt/esp/idf/components/bootloader/subproject"
  "/home/pablo/Documents/esp32-projects/can-gauge/build/bootloader"
  "/home/pablo/Documents/esp32-projects/can-gauge/build/bootloader-prefix"
  "/home/pablo/Documents/esp32-projects/can-gauge/build/bootloader-prefix/tmp"
  "/home/pablo/Documents/esp32-projects/can-gauge/build/bootloader-prefix/src/bootloader-stamp"
  "/home/pablo/Documents/esp32-projects/can-gauge/build/bootloader-prefix/src"
  "/home/pablo/Documents/esp32-projects/can-gauge/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/pablo/Documents/esp32-projects/can-gauge/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/pablo/Documents/esp32-projects/can-gauge/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
