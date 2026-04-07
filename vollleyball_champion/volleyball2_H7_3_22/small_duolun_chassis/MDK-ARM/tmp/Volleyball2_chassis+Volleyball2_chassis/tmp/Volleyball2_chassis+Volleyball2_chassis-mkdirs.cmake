# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "E:/2026volleyball/volleyball/vollleyball_champion/volleyball2_H7/small_duolun_chassis/MDK-ARM/tmp/Volleyball2_chassis+Volleyball2_chassis")
  file(MAKE_DIRECTORY "E:/2026volleyball/volleyball/vollleyball_champion/volleyball2_H7/small_duolun_chassis/MDK-ARM/tmp/Volleyball2_chassis+Volleyball2_chassis")
endif()
file(MAKE_DIRECTORY
  "E:/2026volleyball/volleyball/vollleyball_champion/volleyball2_H7/small_duolun_chassis/MDK-ARM/tmp/1"
  "E:/2026volleyball/volleyball/vollleyball_champion/volleyball2_H7/small_duolun_chassis/MDK-ARM/tmp/Volleyball2_chassis+Volleyball2_chassis"
  "E:/2026volleyball/volleyball/vollleyball_champion/volleyball2_H7/small_duolun_chassis/MDK-ARM/tmp/Volleyball2_chassis+Volleyball2_chassis/tmp"
  "E:/2026volleyball/volleyball/vollleyball_champion/volleyball2_H7/small_duolun_chassis/MDK-ARM/tmp/Volleyball2_chassis+Volleyball2_chassis/src/Volleyball2_chassis+Volleyball2_chassis-stamp"
  "E:/2026volleyball/volleyball/vollleyball_champion/volleyball2_H7/small_duolun_chassis/MDK-ARM/tmp/Volleyball2_chassis+Volleyball2_chassis/src"
  "E:/2026volleyball/volleyball/vollleyball_champion/volleyball2_H7/small_duolun_chassis/MDK-ARM/tmp/Volleyball2_chassis+Volleyball2_chassis/src/Volleyball2_chassis+Volleyball2_chassis-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/2026volleyball/volleyball/vollleyball_champion/volleyball2_H7/small_duolun_chassis/MDK-ARM/tmp/Volleyball2_chassis+Volleyball2_chassis/src/Volleyball2_chassis+Volleyball2_chassis-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/2026volleyball/volleyball/vollleyball_champion/volleyball2_H7/small_duolun_chassis/MDK-ARM/tmp/Volleyball2_chassis+Volleyball2_chassis/src/Volleyball2_chassis+Volleyball2_chassis-stamp${cfgdir}") # cfgdir has leading slash
endif()
