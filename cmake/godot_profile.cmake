if(NOT PROJECT_NAME STREQUAL "godot-cpp")
    return()
endif()

# Pinned godot-cpp rewrites its trimmed API on every configure, forcing a rebuild.
function(build_profile_generate_trimmed_api BUILD_PROFILE INPUT_JSON OUTPUT_JSON)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${BUILD_PROFILE}" "${INPUT_JSON}" "${godot-cpp_SOURCE_DIR}/build_profile.py")
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" "${godot-cpp_SOURCE_DIR}/build_profile.py"
                "${BUILD_PROFILE}" "${INPUT_JSON}" "${OUTPUT_JSON}.tmp"
        WORKING_DIRECTORY "${godot-cpp_SOURCE_DIR}"
        COMMAND_ERROR_IS_FATAL ANY)
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${OUTPUT_JSON}.tmp" "${OUTPUT_JSON}"
        COMMAND_ERROR_IS_FATAL ANY)
    file(REMOVE "${OUTPUT_JSON}.tmp")
endfunction()
