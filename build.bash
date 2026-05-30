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

# On Windows, auto-detect the newest Visual Studio generator using vswhere.
GENERATOR_FLAG=""
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
   VSWHERE="$(cmd //C 'echo %ProgramFiles(x86)%' 2>/dev/null | tr -d '\r')/Microsoft Visual Studio/Installer/vswhere.exe"
   if [[ -f "$VSWHERE" ]]; then
      VS_YEAR=$("$VSWHERE" -latest -property catalog_productLineVersion 2>/dev/null | tr -d '\r')
      case "$VS_YEAR" in
         2022) GENERATOR_FLAG="-G Visual Studio 17 2022" ;;
         2019) GENERATOR_FLAG="-G Visual Studio 16 2019" ;;
         2017) GENERATOR_FLAG="-G Visual Studio 15 2017" ;;
      esac
   fi
   # Fallback: if vswhere not found, try common VS 2022 paths
   if [[ -z "$GENERATOR_FLAG" ]]; then
      for drive in /c /d /e; do
         if [[ -d "$drive/Program Files/Microsoft Visual Studio/2022" ]]; then
            GENERATOR_FLAG="-G Visual Studio 17 2022"
            break
         fi
      done
   fi
fi

if [ -z "$DEBUGOPTSCMAKE" ]; then
   cmake $DEBUGOPTSCMAKE $GENERATOR_FLAG -DFLTK_BUILD_GL=OFF -DCMAKE_BUILD_TYPE=Debug ..
   cmake --build . --config Debug $PARALLEL_FLAG
else
   printf "Redirecting cmake and make outputs to files due to debug mode."
   cmake $DEBUGOPTSCMAKE $GENERATOR_FLAG -DFLTK_BUILD_GL=OFF -DCMAKE_BUILD_TYPE=Debug .. > cmake.log 2> cmake.err
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
