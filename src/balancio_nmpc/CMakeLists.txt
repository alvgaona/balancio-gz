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

# The run-time controller (NMPC + EKF): acados and the generated model C
# only, no CasADi, no ROS. This is what the ESP32 firmware builds.
add_library(balancio_nmpc_core src/nmpc.cpp src/ekf.cpp)
target_include_directories(balancio_nmpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(balancio_nmpc_core PRIVATE acados balancio_model)

# Host side: the CasADi model, code generation and the LQR terminal cost.
add_library(balancio_nmpc src/model.cpp src/codegen.cpp src/lqr.cpp)
target_link_libraries(balancio_nmpc
  PUBLIC balancio_nmpc_core casadi::casadi PRIVATE yaml-cpp::yaml-cpp)

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

# Solve-time benchmark: SQP vs RTI over horizons, in closed loop.
add_executable(bench_nmpc tools/bench_nmpc.cpp)
target_link_libraries(bench_nmpc balancio_nmpc)

# The ROS 2 node, a drop-in for balancio_control's mpc_node.py.
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
add_executable(mpc_node src/mpc_node.cpp)
target_link_libraries(mpc_node balancio_nmpc rclcpp::rclcpp ament_index_cpp::ament_index_cpp
  ${std_msgs_TARGETS} ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS}
  tf2_ros::tf2_ros)
install(TARGETS mpc_node RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(FILES ${GEN_DIR}/balancio_model.c ${GEN_DIR}/balancio_model.h
  DESTINATION include/balancio_nmpc/generated)
install(TARGETS balancio_nmpc balancio_nmpc_core balancio_model balancio_codegen EXPORT balancio_nmpcTargets
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

  # The C++ EKF against the Python EKF on a recorded synthetic run.
  set(EKF_REFERENCE ${CMAKE_CURRENT_BINARY_DIR}/ekf_reference.csv)
  add_test(NAME ekf_reference
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/make_ekf_reference.py
            ${PARAMS_FILE} ${EKF_REFERENCE})
  set_tests_properties(ekf_reference PROPERTIES FIXTURES_SETUP ekf_reference)
  balancio_add_gtest(test_ekf test/test_ekf.cpp)
  target_link_libraries(test_ekf balancio_nmpc)
  target_compile_definitions(test_ekf PRIVATE EKF_REFERENCE="${EKF_REFERENCE}")
  set_tests_properties(test_ekf PROPERTIES FIXTURES_REQUIRED ekf_reference)

  # The generated C against CasADi, and the Jacobian layout acados expects.
  balancio_add_gtest(test_codegen test/test_codegen.cpp)
  target_link_libraries(test_codegen balancio_nmpc balancio_model)
  target_compile_definitions(test_codegen PRIVATE BALANCIO_PARAMS_FILE="${PARAMS_FILE}")
endif()

ament_package()
