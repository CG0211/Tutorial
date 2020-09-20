include_directories(${project_source_dir}/mylib)

aux_source_directory(. DIR_SOURCE_SRC)


add_executable(demo ${DIR_SOURCE_SRC})

target_link_libraries(demo Mylib)
