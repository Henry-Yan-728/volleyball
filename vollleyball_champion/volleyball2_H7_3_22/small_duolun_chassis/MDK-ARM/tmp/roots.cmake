# roots.cmake
set(CMSIS_PACK_ROOT "C:/Users/Lenovo/AppData/Local/Arm/Packs" CACHE PATH "CMSIS pack root")
cmake_path(ABSOLUTE_PATH CMSIS_PACK_ROOT NORMALIZE OUTPUT_VARIABLE CMSIS_PACK_ROOT)
set(CMSIS_COMPILER_ROOT "C:/Users/Lenovo/.vscode/extensions/arm.cmsis-csolution-1.66.0-win32-x64/tools/cmsis-toolbox/etc" CACHE PATH "CMSIS compiler root")
cmake_path(ABSOLUTE_PATH CMSIS_COMPILER_ROOT NORMALIZE OUTPUT_VARIABLE CMSIS_COMPILER_ROOT)
set(SOLUTION_ROOT "E:/2026volleyball/volleyball/vollleyball_champion/volleyball2_H7/small_duolun_chassis/MDK-ARM" CACHE PATH "CMSIS solution root")
cmake_path(ABSOLUTE_PATH SOLUTION_ROOT NORMALIZE OUTPUT_VARIABLE SOLUTION_ROOT)
