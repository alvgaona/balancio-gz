cmake_minimum_required(VERSION 3.16)
project(balancio_nmpc C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(casadi REQUIRED)
find_package(acados REQUIRED)
find_package(yaml-cpp REQUIRED)

set(BALANCIO_PARAMS_FILE
  ${CMAKE_CURRENT_SOURCE_DIR}/../balancio_description/config/balancio.yaml
  CACHE FILEPATH "Physical parameters baked into the generated model")

# The controller library: no ROS, so the ESP32 firmware can use it too.
add_library(balancio_nmpc src/model.cpp src/codegen.cpp src/nmpc.cpp)
target_include_directories(balancio_nmpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(balancio_nmpc
  PUBLIC casadi::casadi PRIVATE yaml-cpp::yaml-cpp acados balancio_model)

# The model functions as plain C, regenerated when the model or the
# parameters change. This is what acados (and later the ESP32) runs.
# Built from the sources, not linked to libbalancio_nmpc: a stale installed
# copy of that library can shadow the fresh one at run time.
add_executable(balancio_codegen tools/codegen.cpp src/model.cpp src/codegen.cpp)
target_include_directories(balancio_codegen PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(balancio_codegen casadi::casadi yaml-cpp::yaml-cpp)
set(GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
add_custom_command(
  OUTPUT ${GEN_DIR}/balancio_model.c ${GEN_DIR}/balancio_model.h
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GEN_DIR}
  COMMAND balancio_codegen ${BALANCIO_PARAMS_FILE} ${GEN_DIR}
  DEPENDS balancio_codegen ${BALANCIO_PARAMS_FILE}
  COMMENT "Generating the model functions in C")
add_library(balancio_model ${GEN_DIR}/balancio_model.c)
target_include_directories(balancio_model PUBLIC
  $<BUILD_INTERFACE:${GEN_DIR}> $<INSTALL_INTERFACE:include/balancio_nmpc/generated>)
# Generated code: unused-parameter warnings are expected.
target_compile_options(balancio_model PRIVATE -O2 -w)

install(DIRECTORY include/ DESTINATION include)
install(FILES ${GEN_DIR}/balancio_model.c ${GEN_DIR}/balancio_model.h
  DESTINATION include/balancio_nmpc/generated)
install(TARGETS balancio_nmpc balancio_model balancio_codegen EXPORT balancio_nmpcTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_export_targets(balancio_nmpcTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(casadi)

if(BUILD_TESTING)
  # conda's GTest, not gtest_vendor: conda's clang puts $CONDA_PREFIX/include
  # first, where conda's gtest headers would shadow the vendored sources.
  find_package(GTest REQUIRED)
  function(balancio_add_gtest name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} GTest::gtest_main)
    add_test(NAME ${name} COMMAND ${name})
  endfunction()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(PARAMS_FILE ${BALANCIO_PARAMS_FILE})
  set(MODEL_REFERENCE ${CMAKE_CURRENT_BINARY_DIR}/model_reference.csv)

  # acados + HPIPM against CasADi + IPOPT on a small MPC QP.
  balancio_add_gtest(test_toolchain test/test_toolchain.cpp)
  target_link_libraries(test_toolchain acados casadi::casadi)

  # The C++ model against the Python model, evaluated fresh each run.
  add_test(NAME model_reference
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/make_model_reference.py
            ${PARAMS_FILE} ${MODEL_REFERENCE})
  set_tests_properties(model_reference PROPERTIES FIXTURES_SETUP model_reference)
  balancio_add_gtest(test_model test/test_model.cpp)
  target_link_libraries(test_model balancio_nmpc)
  target_compile_definitions(test_model PRIVATE
    BALANCIO_PARAMS_FILE="${PARAMS_FILE}" MODEL_REFERENCE="${MODEL_REFERENCE}")
  set_tests_properties(test_model PROPERTIES FIXTURES_REQUIRED model_reference)

  # The acados NMPC against the Python NMPC, and in closed loop.
  set(NMPC_REFERENCE ${CMAKE_CURRENT_BINARY_DIR}/nmpc_reference.csv)
  add_test(NAME nmpc_reference
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/make_nmpc_reference.py
            ${PARAMS_FILE} ${NMPC_REFERENCE})
  set_tests_properties(nmpc_reference PROPERTIES FIXTURES_SETUP nmpc_reference)
  balancio_add_gtest(test_nmpc test/test_nmpc.cpp)
  target_link_libraries(test_nmpc balancio_nmpc)
  target_compile_definitions(test_nmpc PRIVATE
    BALANCIO_PARAMS_FILE="${PARAMS_FILE}" NMPC_REFERENCE="${NMPC_REFERENCE}")
  set_tests_properties(test_nmpc PROPERTIES FIXTURES_REQUIRED nmpc_reference)

  # The generated C against CasADi, and the Jacobian layout acados expects.
  balancio_add_gtest(test_codegen test/test_codegen.cpp)
  target_link_libraries(test_codegen balancio_nmpc balancio_model)
  target_compile_definitions(test_codegen PRIVATE BALANCIO_PARAMS_FILE="${PARAMS_FILE}")
endif()

ament_package()
