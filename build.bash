#!/bin/bash          
# ============================================================================================================
# C O P Y R I G H T
# ------------------------------------------------------------------------------------------------------------
# copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
# ============================================================================================================

reset
clear

DEBUGOPTSCMAKE=""
DEBUGOPTSMAKE=""
RUN_TESTS=false

for var in "$@"
do
  if [[ $var = "-C" || $var = "-c" ]]; then
    rm -r build
    rm -r bin
    rm -r lib
    ccache $var
  fi

  if [[ $var = "-d" ]]; then
    DEBUGOPTSCMAKE="-Wdev --debug-output --trace-expand"
    DEBUGOPTSMAKE="--debug=a"
  fi

  if [[ $var = "-t" ]]; then
    RUN_TESTS=true
  fi
done

mkdir -p build
cd build

# On Windows, auto-detect the Visual Studio generator via vswhere.
# CMake respects the CMAKE_GENERATOR env var, avoiding quoting issues with -G.
# TODO: Find out if this is actually necessary.
if [[ -z "$CMAKE_GENERATOR" ]]; then
   for VSWHERE in \
      "/c/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe" \
      "/d/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe" \
      "/e/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe"; do
      if [[ -f "$VSWHERE" ]]; then
         VS_YEAR=$("$VSWHERE" -latest -property catalog_productLineVersion 2>/dev/null | tr -d '\r')
         case "$VS_YEAR" in
            2022) export CMAKE_GENERATOR="Visual Studio 17 2022" ;;
            2019) export CMAKE_GENERATOR="Visual Studio 16 2019" ;;
            *)
               NEWEST=$(cmake --help 2>/dev/null | grep -o 'Visual Studio [0-9][0-9]* [0-9][0-9]*' | sort -t' ' -k3 -nr | head -1)
               if [[ -n "$NEWEST" ]]; then export CMAKE_GENERATOR="$NEWEST"; fi
               ;;
         esac
         break
      fi
   done
fi
# EO TODO: Find out if this is actually necessary.

if [ -z "$DEBUGOPTSCMAKE" ]; then
   cmake $DEBUGOPTSCMAKE -DFLTK_BUILD_GL=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
   cmake --build . --config Release --parallel 16
else
   printf "Redirecting cmake and make outputs to files due to debug mode."
   cmake $DEBUGOPTSCMAKE -DFLTK_BUILD_GL=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .. > cmake.log 2> cmake.err
   cmake --build . --config Release --parallel 16 > make.log 2> make.err
fi

if [ $? -eq 0 ]; then
  if [ "$RUN_TESTS" = true ]; then

    printf "\r\nBuilt \033[1;32mSUCCESSFULLY\033[1;0m. Running tests...\r\n"
    ctest Test --verbose

    cd .. # coverage relative to root dir
    lcov --directory . --capture --output-file ./build/code_coverage.info -rc lcov_branch_coverage=1
    current_location=$(pwd)
    lcov -r ./build/code_coverage.info /usr/\* "${current_location}/gtest/*" "${current_location}/include/geometry/haru/*" "${current_location}/include/json_parsing/*" -o ./build/code_coverage.info

    genhtml build/code_coverage.info --branch-coverage --output-directory ./build/code_coverage_report/
    # To view what lines are tested and what not, add the "Gcov viewer" VSC plugin and command (Ctrl + Shift + P): "Gcov viewer: show"
  else
    printf "\r\nBuilt \033[1;32mSUCCESSFULLY\033[1;0m.\r\n"
  fi
else
  printf "\r\nBuild \033[1;31mFAILED\033[1;0m.\r\n"
  cd ..
  exit 1
fi

cd ..
