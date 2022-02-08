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

include(ExternalProject)
ExternalProject_Add(libruckig-build
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ruckig
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/ruckig
    INSTALL_COMMAND ""
)

coppeliasim_add_plugin(
    simExtRuckig
    LEGACY
    SOURCES
    simExtRuckig.cpp
)
add_dependencies(simExtRuckig libruckig-build)
target_include_directories(simExtRuckig PRIVATE ruckig/include)
target_link_libraries(simExtRuckig ${CMAKE_CURRENT_BINARY_DIR}/ruckig/${CMAKE_SHARED_LIBRARY_PREFIX}ruckig${CMAKE_SHARED_LIBRARY_SUFFIX})
if(APPLE)
    # for some reason, deploy tool won't copy libruckig.dylib
    coppeliasim_add_library(${CMAKE_CURRENT_BINARY_DIR}/ruckig/libruckig.dylib)
endif(APPLE)
