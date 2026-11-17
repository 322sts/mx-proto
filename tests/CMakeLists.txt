FetchContent_Declare(doctest
  URL https://github.com/doctest/doctest/archive/refs/tags/v2.5.3.tar.gz
  SOURCE_SUBDIR do_not_add) # header only: just download, don't run its CMake
FetchContent_MakeAvailable(doctest)

add_executable(mx_tests test_physics.cpp)
target_link_libraries(mx_tests PRIVATE mx_physics)
target_include_directories(mx_tests PRIVATE ${doctest_SOURCE_DIR})
add_test(NAME mx_tests COMMAND mx_tests)
