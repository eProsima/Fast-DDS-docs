# Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.0)

file(STRINGS
    ${CMAKE_SOURCE_DIR}/../docs/conf.py
    LIB_VERSION_TMP
    REGEX "^version = u'[0-9]+.[0-9]+.[0-9]+'"
    )

string(REGEX REPLACE "^version = u'([0-9]+.[0-9]+.[0-9]+)'"
    "\\1"
    LIB_VERSION_STR
    ${LIB_VERSION_TMP}
    )

project(fastrtps-doc-checker VERSION "${LIB_VERSION_STR}" LANGUAGES C CXX)

########################################################################################################################
# Get documentation git branch
########################################################################################################################
set(ENV_FASTRTPS_BRANCH $ENV{FASTRTPS_BRANCH})
set(FASTRTPS_BRANCH "" CACHE STRING "Fast RTPS repository branch against which test the source code.")

if(NOT "${ENV_FASTRTPS_BRANCH}" STREQUAL "")
    set(REPO_TAG "${ENV_FASTRTPS_BRANCH}")
elseif(NOT "${FASTRTPS_BRANCH}" STREQUAL "")
    set(REPO_TAG "${FASTRTPS_BRANCH}")
else()
    find_package(Git REQUIRED)

    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
        RESULT_VARIABLE RESULT
        OUTPUT_VARIABLE REPO_TAG)
endif()

########################################################################################################################
# Download and compile product
########################################################################################################################
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/external)

set(PRODUCT_CMAKELIST_CONTENT "
    cmake_minimum_required(VERSION 3.0)

    include(ExternalProject)
    ExternalProject_Add(fastrtps
        PREFIX product
        GIT_REPOSITORY https://github.com/eProsima/Fast-RTPS.git
        GIT_TAG  origin/${REPO_TAG}
        GIT_SHALLOW TRUE
        CMAKE_GENERATOR Ninja
        CMAKE_ARGS -DBUILD_SHARED_LIBS=OFF -DEPROSIMA_BUILD=ON -DSECURITY=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo \"-DCMAKE_INSTALL_PREFIX=${PROJECT_BINARY_DIR}/install\"
        UPDATE_COMMAND \"\"
        )
    ")

file(WRITE ${PROJECT_BINARY_DIR}/external/CMakeLists.txt "${PRODUCT_CMAKELIST_CONTENT}")

execute_process(COMMAND ${CMAKE_COMMAND} .
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/external
    )

execute_process(COMMAND ${CMAKE_COMMAND} --build .
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/external
    )

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_BINARY_DIR}/install)

find_package(fastrtps REQUIRED)


########################################################################################################################
# Compile example
########################################################################################################################
add_executable(doctest CodeTester.cpp)
target_include_directories(doctest PRIVATE ${PROJECT_BINARY_DIR}/external/product/src/fastrtps/src
    ${PROJECT_BINARY_DIR}/external/product/src/fastrtps/thirdparty/tinyxml2)
target_link_libraries(doctest fastrtps)

########################################################################################################################
# Execute example
########################################################################################################################
enable_testing()
include(CTest)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/XMLTester.xml
    ${CMAKE_CURRENT_BINARY_DIR}/XMLTester.xml COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/GovernanceTester.xml
    ${CMAKE_CURRENT_BINARY_DIR}/GovernanceTester.xml COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/PermissionsTester.xml
    ${CMAKE_CURRENT_BINARY_DIR}/PermissionsTester.xml COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/StaticTester.xml
    ${CMAKE_CURRENT_BINARY_DIR}/StaticTester.xml COPYONLY)

add_test(NAME xmltest COMMAND doctest XMLTester.xml)
add_test(NAME governancetest COMMAND doctest GovernanceTester.xml)
add_test(NAME permissionstest COMMAND doctest PermissionsTester.xml)
add_test(NAME statictest COMMAND doctest StaticTester.xml)
