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

# Determine parallel build flags based on CMake version.
# --parallel was added in CMake 3.12. Use native flags as fallback.
CMAKE_MAJOR=$(cmake --version | head -1 | sed 's/[^0-9]*//' | cut -d. -f1)
CMAKE_MINOR=$(cmake --version | head -1 | sed 's/[^0-9]*//' | cut -d. -f2)
if [[ "$CMAKE_MAJOR" -gt 3 ]] || [[ "$CMAKE_MAJOR" -eq 3 && "$CMAKE_MINOR" -ge 12 ]]; then
   PARALLEL_FLAG="--parallel 16"
elif [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
   PARALLEL_FLAG="-- //m:16"
else
   PARALLEL_FLAG="-- -j16"
fi

# On Windows, auto-detect the newest Visual Studio generator.
# CMake respects the CMAKE_GENERATOR env var, avoiding quoting issues with -G.
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
   if [[ -z "$CMAKE_GENERATOR" ]]; then
      # Try vswhere at its well-known location
      for VSWHERE in \
         "/c/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe" \
         "/d/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe" \
         "/e/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe"; do
         if [[ -f "$VSWHERE" ]]; then
            VS_YEAR=$("$VSWHERE" -latest -property catalog_productLineVersion 2>/dev/null | tr -d '\r')
            case "$VS_YEAR" in
               2022) export CMAKE_GENERATOR="Visual Studio 17 2022" ;;
               2019) export CMAKE_GENERATOR="Visual Studio 16 2019" ;;
            esac
            break
         fi
      done
      # Fallback: parse cmake --help for the newest VS generator available
      if [[ -z "$CMAKE_GENERATOR" ]]; then
         if cmake --help 2>/dev/null | grep -q "Visual Studio 17 2022"; then
            export CMAKE_GENERATOR="Visual Studio 17 2022"
         elif cmake --help 2>/dev/null | grep -q "Visual Studio 16 2019"; then
            export CMAKE_GENERATOR="Visual Studio 16 2019"
         fi
      fi
   fi
fi

if [ -z "$DEBUGOPTSCMAKE" ]; then
   cmake $DEBUGOPTSCMAKE -DFLTK_BUILD_GL=OFF -DCMAKE_BUILD_TYPE=Debug ..
   cmake --build . --config Debug $PARALLEL_FLAG
else
   printf "Redirecting cmake and make outputs to files due to debug mode."
   cmake $DEBUGOPTSCMAKE -DFLTK_BUILD_GL=OFF -DCMAKE_BUILD_TYPE=Debug .. > cmake.log 2> cmake.err
   cmake --build . --config Debug $PARALLEL_FLAG > make.log 2> make.err
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
