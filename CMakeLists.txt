cmake_minimum_required(VERSION 3.1)

# Disable in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "This project doesn't support in-source builds")
endif()

project(mc_panda_lirmm LANGUAGES CXX VERSION 1.0)
enable_testing()
set(CMAKE_CXX_STANDARD 17)

find_package(mc_panda REQUIRED)

add_subdirectory(modules)
add_subdirectory(cmake)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
