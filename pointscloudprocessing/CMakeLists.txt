cmake_minimum_required(VERSION 3.9)

set(CMAKE_CXX_STANDARD 11)

project(pointscloudprocessing)

set(BOOST_ROOT "C:/PCL 1.12.1/3rdParty/Boost")

find_package(Boost REQUIRED
             COMPONENTS system filesystem thread)

#find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)
find_package(PCL 1.3 REQUIRED)
find_package(VTK REQUIRED)

set(SOURCE_FILES main.cpp)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    )

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${BOOST_LIBRARY_DIRS}
    ${VTK_LIBRARY_DIRS}
    )

add_definitions(
    ${PCL_DEFINITIONS}
    )
add_compile_definitions(NOMINMAX)

add_executable(pointscloudprocessing ${SOURCE_FILES})

target_link_libraries(pointscloudprocessing 
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${VTK_LIBRARIES}
    )