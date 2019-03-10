add_library(library2 SHARED library2.cpp library2.h)

set_target_properties(library2 PROPERTIES
        PUBLIC_HEADER "library2.h")

install(TARGETS library2
        # IMPORTANT: Add the foo library to the "export-set"
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/library2"
        COMPONENT dev)