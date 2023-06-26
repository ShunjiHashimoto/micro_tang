cmake_minimum_required(VERSION 3.20)

project("micro_tang" C CXX ASM)

include(cmake/st-project.cmake)

add_executable(${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/Core/Src/main.c
  ${PROJECT_SOURCE_DIR}/Application/Src/wrapper.cpp
  ${PROJECT_SOURCE_DIR}/Application/Src/led.cpp
  ${PROJECT_SOURCE_DIR}/Application/Src/drivers/motor.cpp
  ${PROJECT_SOURCE_DIR}/Application/Src/pins/timers.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${PROJECT_SOURCE_DIR}/Core/Inc
  ${PROJECT_SOURCE_DIR}/Application/Inc)

add_st_target_properties(${PROJECT_NAME})