#set(DEPENDENCIES realsense2 )
add_library(TdLib SHARED
        threadSafeStructure/threadSafeStructure.cpp
        threadSafeStructure/threadSafeStructure.h
        realsense/realsense.cpp
        realsense/realsense.h
        )

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
##PCL
#find_package(PCL 1.8 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})

set_target_properties(TdLib PROPERTIES
        PUBLIC_HEADER
        "realsense/realsense.h;threadSafeStructure/threadSafeStructure.h")
#        "threadSafeStructure/threadSafeStructure.h")
target_link_libraries(TdLib
#                    ${DEPENDENCIES}
                    ${OpenCV_LIBRARIES}
#                    ${PCL_LIBRARIES}
                    )
install(TARGETS TdLib
        # IMPORTANT: Add the foo library to the "export-set"
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/TdLibrary"
        COMPONENT dev)