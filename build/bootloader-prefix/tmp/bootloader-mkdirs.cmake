# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/esp/esp-idf/components/bootloader/subproject"
  "D:/IMU/IMU_test/build/bootloader"
  "D:/IMU/IMU_test/build/bootloader-prefix"
  "D:/IMU/IMU_test/build/bootloader-prefix/tmp"
  "D:/IMU/IMU_test/build/bootloader-prefix/src/bootloader-stamp"
  "D:/IMU/IMU_test/build/bootloader-prefix/src"
  "D:/IMU/IMU_test/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/IMU/IMU_test/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
