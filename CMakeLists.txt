cmake_minimum_required(VERSION 3.25)
project(mx_proto LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Some fetched deps declare very old cmake_minimum_required; CMake 4 refuses them without this.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

if(MSVC)
  add_compile_options(/utf-8 /MP)
endif()

include(FetchContent)

add_subdirectory(physics)
add_subdirectory(sandbox)

enable_testing()
add_subdirectory(tests)
