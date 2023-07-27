cmake_minimum_required(VERSION 3.9)

set(CMAKE_CXX_STANDARD 11)

project(storepointscloud)

set(BOOST_ROOT "C:/PCL 1.12.1/3rdParty/Boost")

find_package(Boost REQUIRED
             COMPONENTS system filesystem thread)
find_package(PCL 1.3 REQUIRED)

set(SOURCE_FILES main.cpp)

include_directories(Include/)
include_directories(common/include)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

link_directories(Lib/Win64)
link_directories(common/lib/Win64)
link_directories(
    ${PCL_LIBRARY_DIRS}
    ${BOOST_LIBRARY_DIRS}
    )

add_definitions(
    ${PCL_DEFINITIONS}
    )
add_compile_definitions(NOMINMAX)

add_executable(storepointscloud ${SOURCE_FILES})

target_link_libraries(storepointscloud 
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    )

target_link_libraries(storepointscloud Mv3dRgbd glfw3 opengl32 glu32)