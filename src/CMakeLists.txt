add_library(TdLib SHARED
        threadSafeStructure/threadSafeStructure.cpp
        threadSafeStructure/threadSafeStructure.h
        test/library2.cpp
        test/library2.h)

set_target_properties(TdLib PROPERTIES
        PUBLIC_HEADER
        "test/library2.h;threadSafeStructure/threadSafeStructure.h")

install(TARGETS TdLib
        # IMPORTANT: Add the foo library to the "export-set"
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/TdLibrary"
        COMPONENT dev)