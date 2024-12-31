cmake_minimum_required(VERSION 3.22)
project(mobile_robot_algorithms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    models
    scripts
    worlds
  DESTINATION
    share/${PROJECT_NAME}
)

# install(PROGRAMS
#   scripts/repub_odometry_joint_states.py
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
endif()

ament_package()
