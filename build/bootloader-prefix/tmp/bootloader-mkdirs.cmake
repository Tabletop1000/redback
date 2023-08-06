# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/jadon/esp/esp-idf/components/bootloader/subproject"
  "C:/Data/Code/legtest/build/bootloader"
  "C:/Data/Code/legtest/build/bootloader-prefix"
  "C:/Data/Code/legtest/build/bootloader-prefix/tmp"
  "C:/Data/Code/legtest/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Data/Code/legtest/build/bootloader-prefix/src"
  "C:/Data/Code/legtest/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Data/Code/legtest/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Data/Code/legtest/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
