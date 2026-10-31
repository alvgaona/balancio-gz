cmake_minimum_required(VERSION 3.16)
project(balancio_nmpc CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(casadi REQUIRED)
find_package(acados REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # acados + HPIPM against CasADi + IPOPT on a small MPC QP.
  ament_add_gtest(test_toolchain test/test_toolchain.cpp)
  target_link_libraries(test_toolchain acados casadi::casadi)
endif()

ament_package()
