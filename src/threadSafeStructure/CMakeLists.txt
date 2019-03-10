add_library(library1 SHARED library1.cpp library1.h)

set_target_properties(library1 PROPERTIES
        PUBLIC_HEADER "library1.h")

install(TARGETS library1
        # IMPORTANT: Add the foo library to the "export-set"
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/library1"
        COMPONENT dev)