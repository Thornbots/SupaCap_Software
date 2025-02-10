# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/bubba/robomasters/SupaCap_Software/build/_deps/picotool-src"
  "/home/bubba/robomasters/SupaCap_Software/build/_deps/picotool-build"
  "/home/bubba/robomasters/SupaCap_Software/build/_deps"
  "/home/bubba/robomasters/SupaCap_Software/build/picotool/tmp"
  "/home/bubba/robomasters/SupaCap_Software/build/picotool/src/picotoolBuild-stamp"
  "/home/bubba/robomasters/SupaCap_Software/build/picotool/src"
  "/home/bubba/robomasters/SupaCap_Software/build/picotool/src/picotoolBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/bubba/robomasters/SupaCap_Software/build/picotool/src/picotoolBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/bubba/robomasters/SupaCap_Software/build/picotool/src/picotoolBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
