# ============================================================================================================
# C O P Y R I G H T
# ------------------------------------------------------------------------------------------------------------
# copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
# ============================================================================================================

cmake_minimum_required(VERSION 3.11.0)
project(vfm)

# http://patorjk.com/software/taag/#p=display&h=1&c=echo&f=Cricket&t=vfm
message(STATUS "         ___           ")
message(STATUS ".--.--..'  _|.--------.")
message(STATUS "|  |  ||   _||        |")
message(STATUS " \\___/ |__|  |__|__|__|")
message(STATUS "    very fast math")
message(STATUS "                       ")

# The following flags are available. Don't set them here, though! ADD_DEFINITIONS(-DDISALLOW_EXTERNAL_ASSOCIATIONS) #
# Allows to turn off external variables when they are not needed. ADD_DEFINITIONS(-DFSM_DEBUG)
# ADD_DEFINITIONS(-DCOMPOUND_DEBUG) ADD_DEFINITIONS(-DMETA_DEBUG) ADD_DEFINITIONS(-DACTIVATE_CONNECTION_TO_STLLIB)
# ADD_DEFINITIONS(-DCUT_OUT_ALL_DEBUG_OUTPUT_FOR_RELEASE) # This concerns only the failable class, not the above special
# debugs.

# Ensure -fPIC is used for all targets, particularly fltk. TODO: This prohibit compiler optimizations and should go away.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(TESTING_ENABLED
    true
)

if(MSVC)
    set(CMAKE_CXX_STANDARD
        17
    )
    set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} /bigobj /EHc /std:c++latest /Zc:__cplusplus"
    )
endif(MSVC)

SET(GCC_COVERAGE_COMPILE_FLAGS "-g -O0 -coverage -fprofile-arcs -ftest-coverage")
SET(GCC_COVERAGE_LINK_FLAGS    "-coverage -lgcov")
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=300000000"
)

if(UNIX)
    set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -lstdc++fs -Wl,--no-as-needed -ldl")

    message(Unix build)
    set(CMAKE_CXX_STANDARD
        17
    )
    set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} -O3 -fPIC"
    ) # <== OLD
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++fs -Wall -Wextra -Weffc++ -Werror -Wno-non-virtual-dtor
    # -pedantic") # <== BETTER REPLACE WITH
endif(UNIX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin"
)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/lib"
)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin"
)

#add_compile_definitions(ASMJIT_ENABLED)
set(ASMJIT_DIR
   "${CMAKE_CURRENT_SOURCE_DIR}/src/asmjit"
)
include("${ASMJIT_DIR}/CMakeLists.txt")
set(VFM_MAIN_INCLUDE_DIRS
   ${CMAKE_CURRENT_SOURCE_DIR}/src
   ${CMAKE_CURRENT_SOURCE_DIR}/include
#   ${ASMJIT_DIR}/src/asmjit/
)

include_directories(${VFM_MAIN_INCLUDE_DIRS})

set(VFM_INCLUDES_FOR_PARENT
    ${VFM_MAIN_INCLUDE_DIRS}
    PARENT_SCOPE
)

add_subdirectory(src)

if(TESTING_ENABLED
   AND UNIX
)
    enable_testing()
    add_subdirectory(gtest)
endif()
