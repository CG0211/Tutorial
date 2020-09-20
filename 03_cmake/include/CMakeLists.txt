#cmake_minimum_required(VERSION 2.8)
#project(Mylib CXX)

# set(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARAY_DIR}/lib)

aux_source_directory(. LIB_SOURCE)
add_library(Mylib STATIC ${LIB_SOURCE})
target_include_directories(Mylib INTERFACE ${CMAKE_SOURCE_DIR}/include)
