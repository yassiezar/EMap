cmake_minimum_required(VERSION 3.0.0)
project(surface_building VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)

include(CTest)
enable_testing()

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(surface_building src/main.cpp)
target_link_libraries(surface_building ${PCL_LIBRARIES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
