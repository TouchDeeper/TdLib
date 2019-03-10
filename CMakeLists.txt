cmake_minimum_required(VERSION 3.9)
project(TdLib)
set(${PROJECT_NAME}_MAJOR_VERSION 0)
set(${PROJECT_NAME}_MINOR_VERSION 1)
set(${PROJECT_NAME}_PATCH_VERSION 0)
set(${PROJECT_NAME}_VERSION
        ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})

set(CMAKE_CXX_STANDARD 11)
set(DEFAULT_BUILD_TYPE "Release")
message(STATUS "CMAKE_INSTALL_PREFIX don't need the / in the end")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
        "Installation directory for header files")
set(DEF_INSTALL_CMAKE_DIR lib/CMake/${PROJECT_NAME})
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
        "Installation directory for CMake files")
foreach(p LIB BIN INCLUDE CMAKE)
    set(var INSTALL_${p}_DIR)
    if(NOT IS_ABSOLUTE "${${var}}")
        set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
    endif()
endforeach()
message(STATUS "INSTALL_LIB_DIR :=" ${INSTALL_LIB_DIR})
message(STATUS "INSTALL_BIN_DIR :=" ${INSTALL_BIN_DIR})
message(STATUS "INSTALL_INCLUDE_DIR :=" ${INSTALL_INCLUDE_DIR})
message(STATUS "INSTALL_CMAKE_DIR :=" ${INSTALL_CMAKE_DIR})
# set up include-directories
include_directories(
        "${PROJECT_SOURCE_DIR}"   # to find foo/foo.h
        "${PROJECT_BINARY_DIR}")  # to find foo/config.h
message(STATUS "PROJECT_SOURCE_DIR :=" ${PROJECT_SOURCE_DIR})
message(STATUS "PROJECT_BINARY_DIR :=" ${PROJECT_BINARY_DIR})

include(GNUInstallDirs)
add_subdirectory(src)
# Add all targets to the build-tree export set
export(TARGETS library
        FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE ${PROJECT_NAME})

# Create the FooBarConfig.cmake and FooBarConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
        "${INSTALL_INCLUDE_DIR}")
message(STATUS "REL_INCLUDE_DIR :=" ${REL_INCLUDE_DIR})
message(STATUS "INSTALL_INCLUDE_DIR :=" ${INSTALL_INCLUDE_DIR})
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(${PROJECT_NAME}Config.cmake.in
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)

# ... for the install tree
set(CONF_INCLUDE_DIRS "\${TdLib_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(${PROJECT_NAME}Config.cmake.in
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY)
message(STATUS "${TdLib_CMAKE_DIR}/${REL_INCLUDE_DIR}")
# ... for both
configure_file(${PROJECT_NAME}ConfigVersion.cmake.in
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)

# Install the TdLibConfig.cmake and TdLibConfigVersion.cmake
install(FILES
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)
# Install the export set for use with the install-tree
install(EXPORT ${PROJECT_NAME}Targets DESTINATION
        "${INSTALL_CMAKE_DIR}" COMPONENT dev)
