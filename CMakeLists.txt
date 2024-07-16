cmake_minimum_required(VERSION 3.28)
project(SID)

set(CMAKE_CXX_STANDARD 17)
ADD_COMPILE_DEFINITIONS (URDF_RSC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/urdf/")

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
set(RAISIM_PATH "$ENV{HOME}/Library/raisimLib/raisim/linux")
include_directories(${RAISIM_PATH}/include)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${RAISIM_PATH}/lib)

add_executable(SID main.cpp RobotMath.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC raisim pthread)
