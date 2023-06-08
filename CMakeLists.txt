cmake_minimum_required(VERSION 3.15)
project(simRuckig)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED)

if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        set(COPPELIASIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH ${COPPELIASIM_INCLUDE_DIR}/cmake)
find_package(CoppeliaSim 4.5.1.7 REQUIRED)

if(WIN32)
    add_compile_definitions(_USE_MATH_DEFINES)
endif()

include(ExternalProject)
ExternalProject_Add(libruckig-build
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ruckig
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/ruckig
    INSTALL_COMMAND ""
)

if(WIN32)
    set(RUCKIG_LIBRARY_PATH ${CMAKE_CURRENT_BINARY_DIR}/ruckig/Release)
    set(RUCKIG_SHARED_LIBRARY ${RUCKIG_LIBRARY_PATH}/ruckig.dll)
    set(RUCKIG_LINK_LIBRARY ${RUCKIG_LIBRARY_PATH}/ruckig.lib)
else()
    set(RUCKIG_LIBRARY_PATH ${CMAKE_CURRENT_BINARY_DIR}/ruckig)
    set(RUCKIG_SHARED_LIBRARY ${RUCKIG_LIBRARY_PATH}/${CMAKE_SHARED_LIBRARY_PREFIX}ruckig${CMAKE_SHARED_LIBRARY_SUFFIX})
    set(RUCKIG_LINK_LIBRARY ${RUCKIG_SHARED_LIBRARY})
endif()

coppeliasim_add_plugin(
    simRuckig
    LEGACY
    SOURCES
    simRuckig.cpp
)
add_dependencies(simRuckig libruckig-build)
target_include_directories(simRuckig PRIVATE ruckig/include)
target_link_libraries(simRuckig ${RUCKIG_LINK_LIBRARY})
coppeliasim_add_library(${RUCKIG_SHARED_LIBRARY})
