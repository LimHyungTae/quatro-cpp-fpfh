cmake_minimum_required(VERSION 3.10)
project(quatro_cpp_fpfh)

set (CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(OpenMP)
find_package(PCL 1.8 REQUIRED)

option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" ON)
include(3rdparty/find_dependencies.cmake)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        include
        src
)

set(fpfh_src
    src/matcher.cc
    src/fpfh.cc
)

# TEASER++ build test
add_executable(include_test examples/include_test.cc ${fpfh_src})
target_include_directories(include_test
        PUBLIC
        ${PCL_INCLUDE_DIRS})
target_link_libraries(include_test
        Eigen3::Eigen
        teaserpp::teaser_registration
        teaserpp::teaser_io
        TBB::tbb
        ${PCL_LIBRARIES})

add_executable(quatro_cpp_fpfh examples/quatro_cpp_fpfh.cc ${fpfh_src})
target_include_directories(quatro_cpp_fpfh
        PUBLIC
        ${PCL_INCLUDE_DIRS}
        )
target_link_libraries(quatro_cpp_fpfh
        Eigen3::Eigen
        teaserpp::teaser_registration
        teaserpp::teaser_io
        TBB::tbb
        ${PCL_LIBRARIES})

add_executable(quatro_cpp_fpfh_in_kitti examples/quatro_cpp_fpfh_in_kitti.cc ${fpfh_src})
target_include_directories(quatro_cpp_fpfh_in_kitti
        PUBLIC
        ${PCL_INCLUDE_DIRS}
        )
target_link_libraries(quatro_cpp_fpfh_in_kitti
        Eigen3::Eigen
        teaserpp::teaser_registration
        teaserpp::teaser_io
        TBB::tbb
        ${PCL_LIBRARIES})

# Copy the data files to build directory
file(COPY ./example_data/
        DESTINATION ./example_data/
        FILES_MATCHING
        PATTERN *.ply
        PATTERN *.bin)
