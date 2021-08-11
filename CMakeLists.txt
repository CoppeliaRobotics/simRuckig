cmake_minimum_required(VERSION 3.15)
project(simExtRuckig)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED)

if(NOT LIBPLUGIN_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(LIBPLUGIN_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/libPlugin)
    else()
        set(LIBPLUGIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libPlugin)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH ${LIBPLUGIN_DIR}/cmake)
find_package(CoppeliaSim 4.1.0.0 REQUIRED)

add_subdirectory(ruckig)

coppeliasim_add_plugin(
    simExtRuckig
    LEGACY
    SOURCES
    simExtRuckig.cpp
)
target_include_directories(simExtRuckig PRIVATE ruckig/include)
target_link_libraries(simExtRuckig ruckig::ruckig)
