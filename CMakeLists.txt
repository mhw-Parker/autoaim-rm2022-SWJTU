cmake_minimum_required(VERSION 2.9)

project(MASTER)
SET(CMAKE_BUILD_TYPE DEBUG)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -g -std=c++17 -pthread -lv4l1 -lv4l2")
SET(BIN_NAME "run")
SET(CMAKE_CXX_STANDARD 14)
#根据实际路径修改
#set(OpenCV_DIR "/home/tyy/opencv-4.5.4/build")

FIND_PACKAGE(OpenCV 4.5.4 REQUIRED)
FIND_PACKAGE(Threads)
FIND_PACKAGE(Ceres REQUIRED)
#FIND_PACKAGE(glog REQUIRED)

# 加入一个配置头文件，用于处理 CMake 对源码的设置
configure_file (
        "${PROJECT_SOURCE_DIR}/config.h.in"
        "${PROJECT_SOURCE_DIR}/config.h"
)

option (DAHUA "enable dahua" OFF)
option (MIND  "enable mind"  OFF)
option (REALSENSE "enable realsense" OFF)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/Thread)
include_directories(${PROJECT_SOURCE_DIR}/Serials)
include_directories(${PROJECT_SOURCE_DIR}/Detector/Armor/include)
include_directories(${PROJECT_SOURCE_DIR}/Detector/Energy/include)
include_directories(${PROJECT_SOURCE_DIR}/Other/include)
include_directories(${PROJECT_SOURCE_DIR}/Math/Kalman)
include_directories(${PROJECT_SOURCE_DIR}/Math/SolveAngle)
include_directories(${PROJECT_SOURCE_DIR}/Math/Filter)
include_directories(${PROJECT_SOURCE_DIR}/Math/RMKF)
include_directories(${PROJECT_SOURCE_DIR}/Predictor)
include_directories(${PROJECT_SOURCE_DIR}/Drivers/V4L2KAS/include)
include_directories(${PROJECT_SOURCE_DIR}/Drivers/VideoDriver/include)
include_directories(${PROJECT_SOURCE_DIR}/Drivers/ImageWriter/include)
include_directories(${PROJECT_SOURCE_DIR}/Drivers)
include_directories(${PROJECT_SOURCE_DIR}/)
#Predictor/Predictor.h Predictor/Predictor.cpp
if (DAHUA)
    include_directories(${PROJECT_SOURCE_DIR}/Drivers/DAHUA/include)
    link_directories(${PROJECT_SOURCE_DIR}/Drivers/DAHUA/lib)
    link_libraries(MVSDK RecordVideo avcodec avfilter avformat avutil postproc swresample swscale x264 ImageConvert rt pthread)
    file(GLOB_RECURSE DRIVER_SRC "Drivers/DAHUA/src/*.cpp")
endif (DAHUA)

if (MIND)
    include_directories(${PROJECT_SOURCE_DIR}/Drivers/MindVision/include)
    link_libraries(MVSDK)
    file(GLOB_RECURSE DRIVER_SRC "Drivers/MindVision/src/*.cpp")
endif (MIND)

if (REALSENSE)
    FIND_PACKAGE(realsense2 REQUIRED)
    include_directories(${PROJECT_SOURCE_DIR}/Drivers/RealSense/include)
    include_directories(${realsense2_INCLUDE_DIR})
    file(GLOB_RECURSE DRIVER_SRC "Drivers/RealSense/src/*.cpp")
endif (REALSENSE)

include_directories(/usr/include/eigen3)
include_directories(${CERES_INCLUDE_DIRS})

MESSAGE(STATUS "Project:MASTER")
MESSAGE(STATUS "Project Directory: ${PROJECT_SOURCE_DIR}")
MESSAGE(STATUS "OpenCV Version: ${OpenCV_VERSION}")
MESSAGE(STATUS "MindVision CAMERA is : ${MIND}")
MESSAGE(STATUS "DAHUA CAMERA is : ${DAHUA}")
MESSAGE(STATUS "realsense CAMERA is : ${REALSENSE}")
MESSAGE(STATUS "Found Ceres: ${CERES_FOUND}")


FILE(GLOB_RECURSE sourceFiles "Other/src/*.cpp" "Detector/Energy/src/*.cpp" "Detector/Armor/src/*.cpp" "Serials/*.cpp" "Thread/*.cpp"
        "Math/Filter/*.cpp" "Math/SolveAngle/*.cpp" "Math/Kalman/*.cpp" "Drivers/V4L2KAS/src/*.cpp" "Drivers/VideoDriver/src/*.cpp"
        "Drivers/ImageWriter/src/*.cpp" "Math/RMKF/*.cpp" "Predictor/*.cpp")

#when "Drivers/DAHUA/src/*.cpp" is added to the sourcefiles at last line, these .cpp files looks like not been complied
ADD_EXECUTABLE(${BIN_NAME} main.cpp ${sourceFiles} ${DRIVER_SRC} )

list(APPEND PSMOVEAPI_TRACKER_REQUIRED_LIBS v4l2)
SET(LIBS glog pthread)
target_link_libraries(${BIN_NAME} ${LIBS})
TARGET_LINK_LIBRARIES(${BIN_NAME} ${CMAKE_THREAD_LIBS_INIT})
TARGET_LINK_LIBRARIES(${BIN_NAME} ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(${BIN_NAME} Ceres::ceres)
#TARGET_LINK_LIBRARIES(${BIN_NAME} glog::glog)

if(REALSENSE)
    TARGET_LINK_LIBRARIES(${BIN_NAME} ${realsense2_LIBRARY})
endif(REALSENSE)


