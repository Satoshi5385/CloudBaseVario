if(NOT EXISTS "${INPUT_FILE}")
    message(FATAL_ERROR "Application image not found: ${INPUT_FILE}")
endif()

file(SIZE "${INPUT_FILE}" image_size)
if(image_size GREATER MAX_SIZE)
    message(FATAL_ERROR
        "UPDATE.BIN is ${image_size} bytes; maximum is ${MAX_SIZE} bytes")
endif()

execute_process(
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${INPUT_FILE}" "${OUTPUT_FILE}"
    COMMAND_ERROR_IS_FATAL ANY
)
message(STATUS "UPDATE.BIN: ${image_size}/${MAX_SIZE} bytes")
