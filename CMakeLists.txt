project(cvui_program_chooseScreen)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

set(CMAKE_CXX_FLAGS "-std=c++11")
#寻找opencv库
find_package(OpenCV REQUIRED)
#message(STATUS ${OpenCV_INCLUDE_DIRS})
#添加头文件
INCLUDE_DIRECTORIES(./)
include_directories(${OpenCV_INCLUDE_DIRS})
#链接Opencv库
target_link_libraries(cvui_program_chooseScreen ${OpenCV_LIBS} )
#添加后可进行调试
set( CMAKE_BUILD_TYPE Debug )
set(DEPENDENCIES realsense2 )
target_link_libraries(cvui_program_chooseScreen ${DEPENDENCIES})
