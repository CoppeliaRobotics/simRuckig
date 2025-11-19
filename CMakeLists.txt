cmake_minimum_required(VERSION 3.16.3)
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
find_package(CoppeliaSim 4.5.1.7 REQUIRED COMPONENTS simLib-1)

if(WIN32)
    add_compile_definitions(_USE_MATH_DEFINES)
endif()

include(ExternalProject)
ExternalProject_Add(libruckig-build
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/ruckig
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/ruckig
    CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
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
    sourceCode/simRuckig.cpp
)
add_dependencies(simRuckig libruckig-build)
target_include_directories(simRuckig PRIVATE external/ruckig/include)
target_link_libraries(simRuckig PRIVATE ${RUCKIG_LINK_LIBRARY})
coppeliasim_add_library(${RUCKIG_SHARED_LIBRARY})
coppeliasim_add_lua(lua/simRuckig.lua)
