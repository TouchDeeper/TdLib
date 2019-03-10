add_library(library SHARED
        threadSafeStructure/library1.cpp
        threadSafeStructure/library1.h
        test/library2.cpp
        test/library2.h)

set_target_properties(library PROPERTIES
        PUBLIC_HEADER
        "test/library2.h;threadSafeStructure/library1.h")

install(TARGETS library
        # IMPORTANT: Add the foo library to the "export-set"
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/library"
        COMPONENT dev)