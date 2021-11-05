cmake_minimum_required(VERSION 2.8.4)
project(tmotor_test)
add_compile_options(-std=c++11)

option(USE_CAN_PORT "select CAN port" OFF) # or USB port

include_directories(./lib/ ./include)

if(USE_CAN_PORT)
  add_executable(${PROJECT_NAME}
    ./src/socket_can.cpp
    ./test/tmotor_test.cpp
  )
  target_link_libraries(${PROJECT_NAME} -lpthread)
else(USE_CAN_PORT)
  add_executable(${PROJECT_NAME}
    ./src/CANalyst_2.cpp
    ./test/tmotor_test.cpp
  )
  target_link_libraries(${PROJECT_NAME} -lpthread ${CMAKE_CURRENT_SOURCE_DIR}/lib/libcontrolcan.so)
endif(USE_CAN_PORT)

