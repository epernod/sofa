cmake_minimum_required(VERSION 3.22)
project(chainHybrid)

find_package(SceneCreator REQUIRED)
find_package(Sofa.GUI REQUIRED)
find_package(Sofa.Component REQUIRED)

add_executable(${PROJECT_NAME} chainHybrid.cpp)
target_link_libraries(${PROJECT_NAME} SceneCreator Sofa.GUI Sofa.Component)
