# thread safe structure
file(GLOB thread_safe_lib threadSafeStructure/*.cpp threadSafeStructure/*.h threadSafeStructure/*.hpp)
file(GLOB thread_safe_include threadSafeStructure/*.h threadSafeStructure/*.hpp)
# realsense lib
file(GLOB realsense_lib realsense/*.cpp realsense/*.h realsense/*.hpp)
file(GLOB realsense_include realsense/*.h realsense/*.hpp)
# File opertion
file(GLOB file_operation_lib FileOperation/*.cpp FileOperation/*.h FileOperation/*.hpp)
file(GLOB file_operation_include FileOperation/*.h FileOperation/*.hpp)

# sophus_lib
file(GLOB sophus_lib td_sophus/*.cpp td_sophus/*.h td_sophus/*.hpp)
file(GLOB sophus_include td_sophus/*.h td_sophus/*.hpp)

# pcl_lib
file(GLOB pcl_lib PCL/*.cpp PCL/*.h PCL/*.hpp)
file(GLOB pcl_include PCL/*.h PCL/*.hpp)
# GALGO-2.0
file(GLOB GALGO_lib GALGO-2.0/src/*.cpp GALGO-2.0/src/*.h GALGO-2.0/src/*.hpp)
file(GLOB GALGO_include GALGO-2.0/src/*.h GALGO-2.0/src/*.hpp)
# tool
file(GLOB tool_lib tool/*.cpp tool/*.h tool/*.hpp)
file(GLOB tool_include tool/*.h tool/*.hpp)
# openGA
file(GLOB openGA_lib openGA/src/*.cpp openGA/src/*.h openGA/src/*.hpp)
file(GLOB openGA_include openGA/src/*.h openGA/src/*.hpp)
# slam_tool
file(GLOB slam_lib slam_tool/*/*.cpp slam_tool/*/*.h slam_tool/*/*.hpp slam_tool/*.cpp slam_tool/*.hpp slam_tool/*.h)
file(GLOB slam_include slam_tool/*/*.h slam_tool/*/*.hpp slam_tool/*.h slam_tool/*.hpp)
# algorithm_data_structure
file(GLOB algorithm_data_structure_lib algorithm_data_structure/*.cpp algorithm_data_structure/*.hpp algorithm_data_structure/*.h)
file(GLOB algorithm_data_structure_include algorithm_data_structure/*.h algorithm_data_structure/*.hpp slam_tool/utilities.hpp)
# eigen_lib
file(GLOB eigen_lib td_eigen/*.cpp td_eigen/*.hpp td_eigen/*.h)
file(GLOB eigen_include td_eigen/*.h td_eigen/*.hpp)

# ceres_lib
file(GLOB ceres_lib td_ceres/*/*.cpp td_ceres/*/*.hpp td_ceres/*/*.h td_ceres/*.cpp td_ceres/*.h td_ceres/*.hpp  )
file(GLOB ceres_include td_ceres/*/*.h td_ceres/*/*.hpp td_ceres/*/*.h td_ceres/*/*.hpp)

add_library(TdLib SHARED
        ${thread_safe_lib}
        ${realsense_lib}
        ${file_operation_lib}
        ${pcl_lib}
        ${GALGO_lib}
        ${openGA_lib}
        ${tool_lib}
        ${slam_lib}
        ${algorithm_data_structure_lib}
        ${eigen_lib}
        ${sophus_lib}
        ${ceres_lib}
        )
#set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
if(Threads_FOUND)
    MESSAGE(STATUS "Threads_LIBRARIES =  ${Threads_LIBRARIES}")
endif()

set(OpenCV_DIR "/usr/local/share/OpenCV")
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
if(OpenCV_FOUND)
    MESSAGE(STATUS "OPENCV_INCLUDE_DIRS =  ${OPENCV_INCLUDE_DIRS}")
    MESSAGE(STATUS "OpenCV_LIBRARY_DIRS =  ${OpenCV_LIBRARY_DIRS}")
    MESSAGE(STATUS "OpenCV_LIBRARIES =  ${OpenCV_LIBRARIES}")
endif()

# Eigen
include_directories("/usr/local/include/eigen3" )

#PCL
set(PCL_DIR "/home/td/slam/install/pcl/share/pcl-1.9")
find_package(PCL 1.8 REQUIRED)
if(PCL_FOUND)
    MESSAGE(STATUS "PCL_INCLUDE_DIRS =  ${PCL_INCLUDE_DIRS}")
    MESSAGE(STATUS "PCL_LIBRARY_DIRS =  ${PCL_LIBRARY_DIRS}")
    MESSAGE(STATUS "PCL_LIBRARIES =  ${PCL_LIBRARIES}")
    MESSAGE(STATUS "PCL_VERSION =  ${PCL_VERSION}")
endif()
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

#python
include_directories(${PYTHONHOME}/include)
include_directories(${PYTHONHOME}/Lib/site-packages/numpy/core/include)
link_directories(${PYTHONHOME}/libs)
add_definitions(-DMATPLOTLIBCPP_PYTHON_HEADER=Python.h)

#OpenMp
#FIND_PACKAGE( OpenMP REQUIRED)
#if(OPENMP_FOUND)
#    message("OPENMP FOUND")
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#endif()


#Boost
find_package(Boost COMPONENTS system filesystem thread chrono REQUIRED)
#pangolin
#find_package(Pangolin REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS} )

#no template sophus
#find_package(Sophus REQUIRED)
#include_directories(${Sophus_INCLUDE_DIRS})

#template sophus
include_directories(/home/td/slam/install/sophus/include)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
set(HEADER_FILES
        ${realsense_include}
        ${thread_safe_include}
        ${file_operation_include}
        ${pcl_include}
        ${GALGO_include}
        ${openGA_include}
        ${tool_include}
        ${slam_include}
        ${algorithm_data_structure_include}
        ${eigen_include}
        ${sophus_include}
        ${ceres_include}
        )

target_link_libraries(TdLib PRIVATE
                    realsense2
                    ${OpenCV_LIBRARIES}
                    ${PCL_LIBRARIES}
                    ${Boost_LIBRARIES}
                    Threads::Threads
                    )
#set(INSTALL_LIB_DIR "${PROJECT_BINARY_DIR}/lib") # 假设安装目录在编译目录的lib子目录内

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS TdLib
        # IMPORTANT: Add the foo library to the "export-set"
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
#        PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/TdLibrary" COMPONENT dev
        )
set(CMAKE_SOURCE_DIR_BACKUP "${CMAKE_SOURCE_DIR}")
MESSAGE("CMAKE_CURRENT_SOURCE_DIR = " "${CMAKE_CURRENT_SOURCE_DIR}")
MESSAGE("CMAKE_SOURCE_DIR = " "${CMAKE_SOURCE_DIR}")
set(CMAKE_SOURCE_DIR "${CMAKE_SOURCE_DIR}/src")
MESSAGE("CMAKE_SOURCE_DIR = " "${CMAKE_SOURCE_DIR}")
install(DIRECTORY  "${CMAKE_CURRENT_SOURCE_DIR}/" DESTINATION "${INSTALL_INCLUDE_DIR}/TdLibrary"
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
        )
set(CMAKE_SOURCE_DIR  "${CMAKE_SOURCE_DIR_BACKUP}")
# uninstall target
if(NOT TARGET uninstall)
    configure_file(
            "${PROJECT_SOURCE_DIR}/cmake_uninstall.cmake.in"
            "${PROJECT_SOURCE_DIR}/cmake_uninstall.cmake"
            IMMEDIATE @ONLY)

    add_custom_target(uninstall
            COMMAND ${CMAKE_COMMAND} -P ${PROJECT_SOURCE_DIR}/cmake_uninstall.cmake)
endif()
