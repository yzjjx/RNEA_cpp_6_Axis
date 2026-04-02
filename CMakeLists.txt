cmake_minimum_required(VERSION 3.10)
project(pinocchio_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(pinocchio_demo test/test_pinocchio_rnea.cpp)

target_link_libraries(pinocchio_demo PRIVATE
    pinocchio::pinocchio
    Eigen3::Eigen
)

target_compile_definitions(pinocchio_demo PRIVATE
    PINOCCHIO_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)