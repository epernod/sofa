cmake_minimum_required(VERSION 3.22)
project(LeapMotion)

set(LEAPMOTION_MAJOR_VERSION 0)
set(LEAPMOTION_MINOR_VERSION 1)
set(LEAPMOTION_VERSION ${LEAPMOTION_MAJOR_VERSION}.${LEAPMOTION_MINOR_VERSION})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(SofaFramework REQUIRED)
find_package(SofaBase REQUIRED)
find_package(SofaGeneral REQUIRED)
find_package(SofaOpenglVisual REQUIRED)
find_package(Leap REQUIRED)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
set(LEAPMOTION_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src)


set(HEADER_FILES
    initLeapMotion.h
    src/LeapMotionDriver.h
    src/MyListener.h
)


set(SOURCE_FILES
    initLeapMotion.cpp
    src/LeapMotionDriver.cpp
    src/MyListener.cpp
)



add_library(${PROJECT_NAME} SHARED  ${MOC_FILES} ${SOURCE_FILES} ${README_FILES})


target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${LEAPMOTION_INCLUDE_DIRS}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${LEAPMOTION_INCLUDE_DIRS}/..>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${LEAP_INCLUDE_DIR}>")

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${LEAPMOTION_VERSION})

target_link_libraries(${PROJECT_NAME} Sofa.Helper SofaOpenglVisual SofaUserInteraction ${LEAP_LIBRARY})


## Install rules for the library and headers; CMake package configurations files
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR src
    )

