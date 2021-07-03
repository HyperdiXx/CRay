# Project definition and cmake config
cmake_minimum_required(VERSION 3.0.0)
project(Ray LANGUAGES C CXX VERSION 0.1.0)
set(CMAKE_POLICY_DEFAULT_CMP0077 OLD)
set(CMAKE_CXX_STANDARD 17)

# Global variables setup
set(BIN_DIR ${CMAKE_BINARY_DIR}/app)
set(RES_DIR ${CMAKE_SOURCE_DIR}/res)
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(LIB_DIR ${CMAKE_SOURCE_DIR}/lib)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})
file(COPY ${RES_DIR} DESTINATION ${BIN_DIR})

# Define project files and executable
file(GLOB STARTUP_FILE "${SRC_DIR}/main.cpp")
file(GLOB_RECURSE HEADER_FILES "${SRC_DIR}/*.h" "${SRC_DIR}/*.hpp")
add_executable(${PROJECT_NAME} ${STARTUP_FILE} ${HEADER_FILES})


# STB_IMAGE
set(STB_DIR "${LIB_DIR}/stb")
target_include_directories(${PROJECT_NAME} PRIVATE "${STB_DIR}")

#set(LIBS pthread)
#target_link_libraries(${PROJECT_NAME} ${LIBS})