cmake_minimum_required(VERSION 3.0)
project(DBase)

message("${CMAKE_PREFIX_PATH}")
# find_package()

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

link_directories()

aux_source_directory(${CMAKE_SOURCE_DIR} SRC_TEST)

add_executable(Test
    ${SRC_TEST}
)

target_compile_definitions(Test PUBLIC
    _CRT_SECURE_NO_WARNINGS
    NDEBUG
    UNICODE
    CONSOLE
)

target_link_options(Test PUBLIC)

# if install
include(Config.cmake)