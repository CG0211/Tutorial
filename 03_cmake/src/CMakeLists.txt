#include_directories(${project_source_dir}/include)

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARAY_DIR}/bin)

aux_source_directory(. DIR_SOURCE_SRC)
add_executable(demo ${DIR_SOURCE_SRC})
target_link_libraries(demo Mylib)
