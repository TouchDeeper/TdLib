set(DEPENDENCIES realsense2 )
add_library(TdLib SHARED
        threadSafeStructure/threadSafeStructure.cpp
        threadSafeStructure/threadSafeStructure.h
        realsense/realsense.cpp
        realsense/realsense.h
        FileOperation/FileOperation.cpp
        FileOperation/FileOperation.h
        FileOperation/keyboard.cpp
        FileOperation/keyboard.h
        PCL/filter.cpp
        PCL/filter.h
        PCL/enveloping.cpp
        PCL/enveloping.h
        PCL/tools.cpp
        PCL/tools.h
        PCL/common_typedef.h
        PCL/keypoint.cpp
        PCL/keypoint.h
        PCL/global_feeature.cpp
        PCL/global_feature.h
        PCL/local_features.cpp
        PCL/local_features.h
        PCL/Viewer.cpp
        PCL/Viewer.h
        PCL/icp_related.cpp
        PCL/icp_related.h
        GALGO-2.0/src/Chromosome.hpp
        GALGO-2.0/src/Converter.hpp
        GALGO-2.0/src/Evolution.hpp
        GALGO-2.0/src/example.cpp
        GALGO-2.0/src/Galgo.hpp
        GALGO-2.0/src/GeneticAlgorithm.hpp
        GALGO-2.0/src/Parameter.hpp
        GALGO-2.0/src/Population.hpp
        GALGO-2.0/src/Randomize.hpp
        tool/random_tool.hpp
        matplotlib-cpp/matplotlibcpp.h
        openGA/src/openga.hpp
        tool/random_tool.cpp tool/random_tool.h
        tool/tic_toc.h
        slam_tool/slam_tool.cpp slam_tool/slam_tool.h slam_tool/slam_typedef.h
#        slam_tool/draw_tool.cpp slam_tool/draw_tool.h
        slam_tool/backend/loss_function.cc slam_tool/backend/loss_function.h
        slam_tool/backend/eigen_types.h
        slam_tool/motion_transformation.cpp slam_tool/motion_transformation.h
        algorithm_data_structure/sort.cpp algorithm_data_structure/sort.h
        td_eigen/eigen_common_typedef.h
        td_sophus/sophus_typedef.h

        )
set(OpenCV_DIR "/usr/local/share/OpenCV")
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
if(OpenCV_FOUND)
    MESSAGE(STATUS "OPENCV_INCLUDE_DIRS = " ${OPENCV_INCLUDE_DIRS})
    MESSAGE(STATUS "OpenCV_LIBRARY_DIRS = " ${OpenCV_LIBRARY_DIRS})
    MESSAGE(STATUS "OpenCV_LIBRARIES = " ${OpenCV_LIBRARIES})
endif()

# Eigen
include_directories("/usr/include/eigen3" )

#PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

#python
include_directories(${PYTHONHOME}/include)
include_directories(${PYTHONHOME}/Lib/site-packages/numpy/core/include)
link_directories(${PYTHONHOME}/libs)
add_definitions(-DMATPLOTLIBCPP_PYTHON_HEADER=Python.h)

#OpenMp
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#pthread
#set(THREADS_PREFER_PTHREAD_FLAG ON)
#find_package(Threads REQUIRED)

#Boost
find_package(Boost COMPONENTS system filesystem thread chrono REQUIRED)
#pangolin
#find_package(Pangolin REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS} )
#sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

set(HEADER_FILES
        FileOperation/FileOperation.h
        realsense/realsense.h
        threadSafeStructure/threadSafeStructure.h
        FileOperation/keyboard.h
        PCL/filter.h
        PCL/enveloping.h
        PCL/tools.h
        PCL/common_typedef.h
        PCL/keypoint.h
        PCL/global_feature.h
        PCL/local_features.h
        PCL/Viewer.h
        PCL/icp_related.h
        GALGO-2.0/src/Chromosome.hpp
        GALGO-2.0/src/Converter.hpp
        GALGO-2.0/src/Evolution.hpp
        GALGO-2.0/src/Galgo.hpp
        GALGO-2.0/src/GeneticAlgorithm.hpp
        GALGO-2.0/src/Parameter.hpp
        GALGO-2.0/src/Population.hpp
        GALGO-2.0/src/Randomize.hpp
        tool/random_tool.hpp
        matplotlib-cpp/matplotlibcpp.h
        openGA/src/openga.hpp
        tool/random_tool.h
        tool/tic_toc.h
        slam_tool/slam_tool.h
        slam_tool/slam_typedef.h
#        slam_tool/draw_tool.h
        slam_tool/backend/loss_function.h
        slam_tool/backend/eigen_types.h
        algorithm_data_structure/sort.h
        slam_tool/motion_transformation.h
        td_eigen/eigen_common_typedef.h
        td_sophus/sophus_typedef.h;
        )
set_target_properties(TdLib PROPERTIES
        PUBLIC_HEADER
        "${HEADER_FILES}"
        )

target_link_libraries(TdLib PRIVATE
                    ${DEPENDENCIES}
                    ${OpenCV_LIBRARIES}
                    ${PCL_LIBRARIES}
#                    ${Pangolin_LIBRARIES}
                    ${Sophus_LIBRARIES}
#                    Threads::Threads
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
        PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/TdLibrary"
        COMPONENT dev)
# uninstall target
if(NOT TARGET uninstall)
    configure_file(
            "${PROJECT_SOURCE_DIR}/cmake_uninstall.cmake.in"
            "${PROJECT_SOURCE_DIR}/cmake_uninstall.cmake"
            IMMEDIATE @ONLY)

    add_custom_target(uninstall
            COMMAND ${CMAKE_COMMAND} -P ${PROJECT_SOURCE_DIR}/cmake_uninstall.cmake)
endif()
