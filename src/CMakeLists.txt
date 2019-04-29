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
        PCL/global_feature.h)
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
include_directories( "/usr/include/eigen3" )

#PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
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
        )
set_target_properties(TdLib PROPERTIES
        PUBLIC_HEADER
        "${HEADER_FILES}"
        )

target_link_libraries(TdLib
                    ${DEPENDENCIES}
                    ${OpenCV_LIBRARIES}
                    ${PCL_LIBRARIES}
                    )

install(TARGETS TdLib
        # IMPORTANT: Add the foo library to the "export-set"
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/TdLibrary"
        COMPONENT dev)