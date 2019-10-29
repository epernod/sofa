cmake_minimum_required(VERSION 3.1)
project(SofaCarving VERSION 0.1)

find_package(SofaGeneral REQUIRED)

set(SOFACARVING_SRC_DIR src/SofaCarving)
set(HEADER_FILES
    ${SOFACARVING_SRC_DIR}/config.h.in
    ${SOFACARVING_SRC_DIR}/CarvingManager.h    
)

set(SOURCE_FILES
    ${SOFACARVING_SRC_DIR}/CarvingManager.cpp
    ${SOFACARVING_SRC_DIR}/initSofaCarving.cpp
)

set(README_FILES SofaCarving.txt)

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Set define dllimport/dllexport mechanism on Windows.
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_SOFACARVING")

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} SofaComponentGeneral)

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SOFACARVING")
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

if(SOFA_BUILD_TESTS)
    find_package(SofaTest QUIET)
    if(SofaTest_FOUND)
        add_subdirectory(SofaCarving_test)
    endif()
endif()

## Install rules for the library; CMake package configurations files
sofa_generate_package(
    NAME ${PROJECT_NAME}
    TARGETS ${PROJECT_NAME}
    VERSION ${PROJECT_VERSION}
    INCLUDE_SOURCE_DIR ${SOFACARVING_SRC_DIR}
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )