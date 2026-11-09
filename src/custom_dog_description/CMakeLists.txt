cmake_minimum_required(VERSION 3.8)
project(custom_dog_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md README_EN.md LICENSE.upstream
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(check_model_resources
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/check_model_resources.py"
      "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 30
  )
endif()

ament_package()
