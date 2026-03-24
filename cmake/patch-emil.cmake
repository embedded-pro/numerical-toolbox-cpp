set(EMIL_CMAKELISTS "${emil_SOURCE_DIR}/CMakeLists.txt")
set(EMIL_EXTERNAL_CMAKELISTS "${emil_SOURCE_DIR}/external/CMakeLists.txt")

file(READ "${EMIL_CMAKELISTS}" content)

if (NOT content MATCHES "EMIL_INCLUDE_ECHO")
    string(REPLACE
        "add_subdirectory(protobuf)\nadd_subdirectory(services)\nadd_subdirectory(upgrade)"
        "if (EMIL_INCLUDE_ECHO)\n    add_subdirectory(protobuf)\n    add_subdirectory(services)\n    add_subdirectory(upgrade)\nendif()"
        content "${content}"
    )
    file(WRITE "${EMIL_CMAKELISTS}" "${content}")
endif()

file(READ "${EMIL_EXTERNAL_CMAKELISTS}" ext_content)

if (NOT ext_content MATCHES "EMIL_INCLUDE_ECHO")
    string(REPLACE
        "add_subdirectory(protobuf)\nadd_subdirectory(protoc)"
        "if (EMIL_INCLUDE_ECHO)\n    add_subdirectory(protobuf)\n    add_subdirectory(protoc)\nendif()"
        ext_content "${ext_content}"
    )
    file(WRITE "${EMIL_EXTERNAL_CMAKELISTS}" "${ext_content}")
endif()
