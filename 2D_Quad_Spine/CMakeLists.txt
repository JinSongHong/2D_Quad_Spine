# CMake 프로그램의 최소 버전
cmake_minimum_required(VERSION 3.11)

# 프로젝트 정보
project(
  JinSong
  VERSION 0.1
  DESCRIPTION "Riemannian simulation"
  LANGUAGES CXX)

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

file(GLOB_RECURSE SOURCES "main.cc" "simulate.cpp" "glfw_adapter.cpp" "glfw_dispatch.cpp" "lodepng.cpp" "platform_ui_adapter.cpp"
"Controller.cc" "Kinematics.cc" "Actuator.cc" "filter.cc" "Trajectory.cc" "Integrate.cc")

# add_executable(main ${SOURCES})
# set(SOURCES
#     main.cc
# )

add_executable (main ${SOURCES})
# add_executable (main)
# add_executable (main ${SRC_FILES})




target_include_directories(main PUBLIC
${CMAKE_SOURCE_DIR}/include
${CMAKE_SOURCE_DIR}/Robot

${CMAKE_SOURCE_DIR}/util
${CMAKE_SOURCE_DIR}/../../include
${CMAKE_SOURCE_DIR}/GUI
/usr/include
# /home/jinsong/casadi/build/lib
# /home/jinsong/casadi/build
# /home/jinsong/casadi
# /home/jinsong/lib/Ipopt/src/Interfaces
# /home/jinsong/lib/Ipopt/src/Common
# /home/jinsong/lib/Ipopt/src/LinAlg
/usr/include/eigen-master/unsupported/Eigen

  )

target_link_directories(main PUBLIC           #Library directory
    ${CMAKE_SOURCE_DIR}/../../lib
    
)

target_link_libraries(main
    lapack
    blas
    # casadi
    # ipopt
    mujoco
    glfw
    
)