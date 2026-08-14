# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS "SRC")
set(COMPONENTS SRC espcoredump)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(CloudBaseVario-Aohazuku)

# The only image accepted as /UPDATE.BIN is the raw ESP-IDF application
# binary. Generate a clearly named, size-checked copy beside the normal image.
idf_build_get_property(CBV_PROJECT_BIN PROJECT_BIN)
set(CBV_UPDATE_INPUT "${CMAKE_BINARY_DIR}/${CBV_PROJECT_BIN}")
set(CBV_UPDATE_OUTPUT "${CMAKE_BINARY_DIR}/UPDATE.BIN")
add_custom_target(
    update_bin ALL
    COMMAND "${CMAKE_COMMAND}"
        "-DINPUT_FILE=${CBV_UPDATE_INPUT}"
        "-DOUTPUT_FILE=${CBV_UPDATE_OUTPUT}"
        -DMAX_SIZE=3670016
        -P "${CMAKE_SOURCE_DIR}/tools/check_update_image.cmake"
    DEPENDS "gen_${PROJECT_NAME}_binary"
    COMMENT "Generating size-checked UPDATE.BIN"
    VERBATIM
)
