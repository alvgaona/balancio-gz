cmake_minimum_required(VERSION 3.16)
project(balancio_nmpc CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(casadi REQUIRED)
find_package(acados REQUIRED)
find_package(yaml-cpp REQUIRED)

# The controller library: no ROS, so the ESP32 firmware can use it too.
add_library(balancio_nmpc src/model.cpp)
target_include_directories(balancio_nmpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(balancio_nmpc PUBLIC casadi::casadi PRIVATE yaml-cpp::yaml-cpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS balancio_nmpc EXPORT balancio_nmpcTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(balancio_nmpcTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(casadi)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(PARAMS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../balancio_description/config/balancio.yaml)
  set(MODEL_REFERENCE ${CMAKE_CURRENT_BINARY_DIR}/model_reference.csv)

  # acados + HPIPM against CasADi + IPOPT on a small MPC QP.
  ament_add_gtest(test_toolchain test/test_toolchain.cpp)
  target_link_libraries(test_toolchain acados casadi::casadi)

  # The C++ model against the Python model, evaluated fresh each run.
  add_test(NAME model_reference
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/make_model_reference.py
            ${PARAMS_FILE} ${MODEL_REFERENCE})
  set_tests_properties(model_reference PROPERTIES FIXTURES_SETUP model_reference)
  ament_add_gtest(test_model test/test_model.cpp)
  target_link_libraries(test_model balancio_nmpc)
  target_compile_definitions(test_model PRIVATE
    BALANCIO_PARAMS_FILE="${PARAMS_FILE}" MODEL_REFERENCE="${MODEL_REFERENCE}")
  set_tests_properties(test_model PROPERTIES FIXTURES_REQUIRED model_reference)
endif()

ament_package()
