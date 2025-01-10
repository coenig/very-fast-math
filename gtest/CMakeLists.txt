

CMAKE_MINIMUM_REQUIRED(VERSION 3.11.0)


# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})



#add_executable (TEST_EXECUTABLE google_test.cpp)

add_executable(runTests our_sample_test.cpp)


#TEST_EXECUTABLE
target_link_libraries (runTests
                    ${CMAKE_SOURCE_DIR}/lib/libasmjit.so
                    VFM_MAIN_LIB
                    )


target_link_libraries(runTests ${GTEST_LIBRARIES} pthread)


add_test (NAME OurUnitTests COMMAND runTests)




# project(vfmGtests)

# cmake_minimum_required(VERSION 2.6)

# get_filename_component(PARENT_DIR ../ ABSOLUTE)
# include_directories(${PARENT_DIR}/src)
# include_directories(${PARENT_DIR}/include)

# # Locate GTest
# find_package(GTest REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})

# # Link runTests with what we want to test and the GTest and pthread library
# add_executable(runTests our_sample_test.cpp)
# target_link_libraries(runTests ${GTEST_LIBRARIES} pthread)

