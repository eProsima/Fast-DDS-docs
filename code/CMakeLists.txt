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

find_package(Git REQUIRED)

if(NOT "${ENV_FASTRTPS_BRANCH}" STREQUAL "")
    set(REPO_TAG "${ENV_FASTRTPS_BRANCH}")
elseif(NOT "${FASTRTPS_BRANCH}" STREQUAL "")
    set(REPO_TAG "${FASTRTPS_BRANCH}")
else()
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
        RESULT_VARIABLE RESULT
        OUTPUT_VARIABLE REPO_TAG)
endif()

message("Checking fastrtps branch ${REPO_TAG}...")
execute_process(
    COMMAND ${GIT_EXECUTABLE} ls-remote https://github.com/eProsima/Fast-RTPS.git ${REPO_TAG}
    RESULT_VARIABLE RESULT_LS_REMOTE
    OUTPUT_VARIABLE OUTPUT_LS_REMOTE)

if("${OUTPUT_LS_REMOTE}" STREQUAL "")
    message("fastrtps branch ${REPO_TAG} not found. Changing to master branch.")
    set(REPO_TAG "master")
endif()

########################################################################################################################
# Download and compile product
########################################################################################################################

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/external)

set(PRODUCT_CMAKELIST_CONTENT "
    cmake_minimum_required(VERSION 3.0)

    include(ExternalProject)
    ExternalProject_Add(fastrtps
        PREFIX eprosima
        GIT_REPOSITORY https://github.com/eProsima/Fast-RTPS.git
        GIT_TAG  origin/${REPO_TAG}
        GIT_SHALLOW TRUE
        CONFIGURE_COMMAND \"\"
        BUILD_COMMAND \"\"
        INSTALL_COMMAND \"\"
        UPDATE_COMMAND \"\"
        )
    ExternalProject_Add(buildcop
        PREFIX eprosima
        DOWNLOAD_DIR \"eprosima\"
        URL https://gitlab.intranet.eprosima.com/eProsima/infrastructure/raw/master/jenkins/buildcop/buildcop.py
        DOWNLOAD_NO_EXTRACT 1
        TLS_VERIFY FALSE
        CONFIGURE_COMMAND \"\"
        BINARY_DIR \".\"
        BUILD_COMMAND cd eprosima && python3 buildcop.py --ci-repo https://gitlab.intranet.eprosima.com/eProsima/fastrtps-ci.git --source-branch ${REPO_TAG} --build-args --build-base build --merge-install --mixin rel-with-deb-info --packages-up-to fastrtps --metas ../../.. --cmake-args -DSECURITY=ON --- --no-test
        INSTALL_COMMAND \"\"
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

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${PROJECT_BINARY_DIR}/external/eprosima/install")

find_package(fastrtps REQUIRED)


########################################################################################################################
# Compile example
########################################################################################################################
add_executable(doctest CodeTester.cpp)
target_include_directories(doctest PRIVATE ${PROJECT_BINARY_DIR}/external/eprosima/src/fastrtps/src/cpp
    ${PROJECT_BINARY_DIR}/external/eprosima/src/fastrtps/thirdparty/tinyxml2)
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
set_property(TEST xmltest APPEND PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}/external/eprosima/install/lib")
add_test(NAME governancetest COMMAND doctest GovernanceTester.xml)
set_property(TEST governancetest APPEND PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}/external/eprosima/install/lib")
add_test(NAME permissionstest COMMAND doctest PermissionsTester.xml)
set_property(TEST permissionstest APPEND PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}/external/eprosima/install/lib")
add_test(NAME statictest COMMAND doctest StaticTester.xml)
set_property(TEST statictest APPEND PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}/external/eprosima/install/lib")
