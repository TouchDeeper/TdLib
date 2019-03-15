cmake_minimum_required(VERSION 3.9)
project(LibTest)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
set(CMAKE_CXX_STANDARD 11)
#TdLib
set(TdLib_DIR "/home/wang/CLionProjects/TdLibrary/install/lib/CMake/TdLib/")
find_package(TdLib REQUIRED)
include_directories(${TdLib_INCLUDE_DIRS})
MESSAGE(STATUS "TdLib_INCLUDE_DIRS = " ${TdLib_INCLUDE_DIRS})

#PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
if(OpenCV_FOUND)
    MESSAGE(STATUS "OPENCV_INCLUDE_DIRS = " ${OPENCV_INCLUDE_DIRS})
    MESSAGE(STATUS "OpenCV_LIBRARY_DIRS = " ${OpenCV_LIBRARY_DIRS})
    MESSAGE(STATUS "OpenCV_LIBRARIES = " ${OpenCV_LIBRARIES})
endif()
add_executable(LibTest main.cpp)
message(STATUS "TdLib_LIBRARIES = " ${TdLib_LIBRARIES})
target_link_libraries(LibTest ${TdLib_LIBRARIES}
        Threads::Threads
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        )