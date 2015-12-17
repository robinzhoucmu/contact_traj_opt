cmake_minimum_required(VERSION 3.4)
set(CMAKE_CXX_FLAGS -std=c++11)
file(GLOB tutorials_src "*.cpp")
list(SORT tutorials_src)
list(APPEND CMAKE_PACKAGE_PATH "~/dart_modified/build/cmake")



find_package(Dart)
include_directories(${DART_INCLUDE_DIRS})
#include_directories("/usr/local/include")
message(STATUS "${DART_INCLUDE_DIRS}")
message(STATUS "[ Tutorials ]")
message(STATUS "${CMAKE_PACKAGE_PATH}")

add_executable(demo tutorialdemo_leverup_wall.cpp dart-interface.cpp env-model.cpp)

target_link_libraries(demo dart)


