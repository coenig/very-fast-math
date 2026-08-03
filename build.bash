#!/bin/bash          
# ============================================================================================================
# C O P Y R I G H T
# ------------------------------------------------------------------------------------------------------------
# copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
# ============================================================================================================

reset
clear

# Prefer a repo-local CMake upgrade (no admin rights needed) when available.
CMAKE_BIN="cmake"
LOCAL_CMAKE="/c/eig/very-fast-math/tools/cmake-4.3.3-windows-x86_64/bin/cmake"
if [[ -x "$LOCAL_CMAKE" ]]; then
  CMAKE_BIN="$LOCAL_CMAKE"
fi

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

# On Windows, force the MSVC toolchain with the Ninja generator. This is
# independent of the installed Visual Studio version (VS 17, 18, ...) and of
# whether CMake knows a matching "Visual Studio NN" generator. Without this,
# CMake auto-picks the first compiler on PATH, which may be an unrelated MinGW
# g++ (e.g. from Strawberry Perl) that cannot build this project.
if [[ -z "$CMAKE_GENERATOR" && -z "$VCINSTALLDIR" ]]; then
  VS_INSTALL_PATH=""
  for VSWHERE in \
     "/c/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe" \
     "/d/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe" \
     "/e/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe"; do
    [[ -f "$VSWHERE" ]] || continue
    VS_INSTALL_PATH=$("$VSWHERE" -latest -products '*' -property installationPath 2>/dev/null | tr -d '\r')
    [[ -n "$VS_INSTALL_PATH" ]] && break
  done

  if [[ -z "$VS_INSTALL_PATH" ]]; then
    echo "Warning: No Visual Studio installation found via vswhere; falling back to CMake defaults."
  else
    VCVARS_WIN=$(cygpath -w "$VS_INSTALL_PATH/VC/Auxiliary/Build/vcvars64.bat")
    echo "Setting up MSVC (cl.exe) environment from: $VS_INSTALL_PATH"

    # Import the MSVC environment produced by vcvars64.bat into this shell so
    # that cl.exe is on PATH and INCLUDE/LIB/LIBPATH are set for Ninja.
    # We run vcvars via a temporary batch wrapper because Git Bash mangles the
    # quoting needed to call a space-containing path directly through "cmd //c".
    # Note: cmd's "set" emits the path variable as "Path" (mixed case) and uses
    # CRLF line endings, so we upper-case the key for matching and strip the \r.
    VCVARS_TMP_BAT=$(mktemp --suffix=.bat)
    printf '@echo off\r\ncall "%s" >nul 2>&1\r\nset\r\n' "$VCVARS_WIN" > "$VCVARS_TMP_BAT"
    while IFS='=' read -r key value; do
      value="${value%$'\r'}"
      case "${key^^}" in
        PATH)                export PATH="$(cygpath -up "$value")" ;;
        INCLUDE|LIB|LIBPATH) export "${key^^}=$value" ;;
      esac
    done < <(cmd //c "$(cygpath -w "$VCVARS_TMP_BAT")")
    rm -f "$VCVARS_TMP_BAT"

    export CC=cl
    export CXX=cl
    export CMAKE_GENERATOR="Ninja"
  fi
fi

if [ -z "$DEBUGOPTSCMAKE" ]; then
  $CMAKE_BIN $DEBUGOPTSCMAKE -DFLTK_BUILD_GL=OFF -DCMAKE_BUILD_TYPE=Release ..
  $CMAKE_BIN --build . --config Release --parallel 16
else
   printf "Redirecting cmake and make outputs to files due to debug mode."
  $CMAKE_BIN $DEBUGOPTSCMAKE -DFLTK_BUILD_GL=OFF -DCMAKE_BUILD_TYPE=Release .. > cmake.log 2> cmake.err
  $CMAKE_BIN --build . --config Release --parallel 16 > make.log 2> make.err
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
