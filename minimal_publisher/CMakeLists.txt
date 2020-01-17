cmake_minimum_required(VERSION 3.5)
project(minimal_publisher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

set(executable_name minimal_publisher)
set(library_name ${executable_name}_core)
set(dependencies
  rclcpp
  std_msgs)

add_library(${library_name} SHARED
  src/minimal_publisher.cpp)
ament_target_dependencies(${library_name}
  ${dependencies})

add_executable(${executable_name}
  src/main.cpp)
target_link_libraries(${executable_name}
  ${library_name})
ament_target_dependencies(${executable_name}
  ${dependencies})

install(TARGETS
  ${executable_name}
  ${library_name}

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()