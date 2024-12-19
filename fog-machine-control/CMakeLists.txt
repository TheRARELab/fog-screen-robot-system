cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(fog-screen VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the Boost libraries
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Include directories
include_directories(${Boost_INCLUDE_DIRS})

# Add executable
add_executable(SerialCommunication SerialCommunication.c++)

# Link libraries
target_link_libraries(SerialCommunication ${Boost_LIBRARIES})

# Compiler Warnings
target_compile_options(SerialCommunication PRIVATE -Wall -Wextra -pthread)