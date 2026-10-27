cmake_minimum_required(VERSION 3.8)
project(smach_handover_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  smach_handover_ctrl/controller_server.py
  smach_handover_ctrl/planner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()