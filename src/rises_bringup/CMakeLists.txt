cmake_minimum_required(VERSION 3.22)
project(rises_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(rises_bringup)

# Install launch files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
endif()

# Install rviz config files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz)
endif()

# Install config files (QoS overrides, parameters, etc.)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_descriptions test/test_launch_descriptions.py)
  ament_add_pytest_test(test_aux_helpers test/test_aux_helpers.py)
  ament_add_pytest_test(test_params_consistency test/test_params_consistency.py)
endif()

ament_package()
