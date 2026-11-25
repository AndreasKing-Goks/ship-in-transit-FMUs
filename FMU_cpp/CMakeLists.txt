cmake_minimum_required(VERSION 3.23)
project(ship-in-transit-FMUs-cpp)

include(${CMAKE_BINARY_DIR}/generators/conan_toolchain.cmake)

include(fmiCpp)
fmiCppFmu()

add_subdirectory(simplified_ship)
add_subdirectory(mission_manager)
add_subdirectory(ship)
add_subdirectory(env_loads)
