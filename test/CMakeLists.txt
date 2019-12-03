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
# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )
#
## OpenCV
#find_package(OpenCV REQUIRED)
#include_directories(${OPENCV_INCLUDE_DIRS})
#link_directories(${OpenCV_LIBRARY_DIRS})
#if(OpenCV_FOUND)
#    MESSAGE(STATUS "OPENCV_INCLUDE_DIRS = " ${OPENCV_INCLUDE_DIRS})
#    MESSAGE(STATUS "OpenCV_LIBRARY_DIRS = " ${OpenCV_LIBRARY_DIRS})
#    MESSAGE(STATUS "OpenCV_LIBRARIES = " ${OpenCV_LIBRARIES})
#endif()

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
#sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(LibTest main.cpp)

#add_executable(ga_example ga_example.cpp)
message(STATUS "TdLib_LIBRARIES = " ${TdLib_LIBRARIES})
target_link_libraries(LibTest ${TdLib_LIBRARIES}
        ${Sophus_LIBRARIES}
#        Threads::Threads
#        ${OpenCV_LIBRARIES}
#        ${PCL_LIBRARIES}
        )

add_executable(sophus_test sophus_test.cpp)
target_link_libraries(sophus_test ${TdLib_LIBRARIES}
        ${Sophus_LIBRARIES})

add_executable(dynamic_iter dynamic_iter.cpp)

#target_link_libraries(ga_example ${TdLib_LIBRARIES}
#        )
#add_executable(basic basic.cpp)
#target_link_libraries(basic ${TdLib_LIBRARIES}
#        )
#add_executable(example_so1 openGA/example_so1.cpp openGA/openga_class.cpp openGA/openga_class.h)
#target_link_libraries(example_so1 ${TdLib_LIBRARIES}
#        )
#add_executable(openga_class_test openGA/openga_class_test.cpp openGA/openga_class.cpp openGA/openga_class.h)
#target_link_libraries(openga_class_test ${TdLib_LIBRARIES})
#
#add_executable(random_test random_test.cpp)
#target_link_libraries(random_test ${TdLib_LIBRARIES})

#find_package(Pangolin REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS})
#add_executable(pangolin_test pangolin_test.cpp)
#target_link_libraries(pangolin_test ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES} ${TdLib_LIBRARIES})

#add_executable(openMp_test openMp_test/openMp_test.cpp)