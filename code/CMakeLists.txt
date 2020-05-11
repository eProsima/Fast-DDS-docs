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

cmake_minimum_required(VERSION 3.12)

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
# Get fastrtps git branch
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

# Remove trailing endline if present
string(REGEX REPLACE "\n$" "" REPO_TAG "${REPO_TAG}")

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
# Download and compile fastrtps
########################################################################################################################

# Create directory fo external project
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/external)

# CMakeLists.txt for downloading and compiling Fast RTPS
set(PRODUCT_CMAKELIST_CONTENT "
    cmake_minimum_required(VERSION 3.0)
    project(fastrtps-doc-external)
    include(ExternalProject)
    option(BUILDCOP \"Download buildcop.py\" OFF)
    set(BUILDCOP ${BUILDCOP})

    # Download Fast RTPS in same src directory used by buildcop
    ExternalProject_Add(fastrtps
        PREFIX eprosima
        DOWNLOAD_DIR \"eprosima/src\"
        GIT_REPOSITORY https://github.com/eProsima/Fast-RTPS.git
        GIT_TAG origin/${REPO_TAG}
        GIT_SHALLOW TRUE
        CONFIGURE_COMMAND \"\"
        BUILD_COMMAND \"\"
        INSTALL_COMMAND \"\"
        UPDATE_COMMAND \"\"
        )

    if(BUILDCOP)
        # Download and compile Fast RTPS dependencies using buildcop.py
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
            DEPENDS fastrtps
            )
    endif()
    ")
file(WRITE ${PROJECT_BINARY_DIR}/external/CMakeLists.txt "${PRODUCT_CMAKELIST_CONTENT}")

option(BUILDCOP "Activate actual fastrtps building" OFF)
set(BUILDCOP ${BUILDCOP})
# Configure external with BUILDCOP option
execute_process(COMMAND ${CMAKE_COMMAND} . -DBUILDCOP=${BUILDCOP}
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/external
    )
# Build external
execute_process(COMMAND ${CMAKE_COMMAND} --build .
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/external
    )

#########################################################################################################################
# Build Doxygen documentation
#########################################################################################################################

find_package(Doxygen REQUIRED)
# Create doxygen directories
add_custom_target(doc-dirs
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/doxygen
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/html
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/html/doxygen
    COMMENT "Creating documentation directories" VERBATIM)

# Get header files from repo instead of from installation. This way Fast RTPS does not need to be compiled
file(GLOB_RECURSE HPP_FILES "${PROJECT_BINARY_DIR}/external/eprosima/src/fastrtps/include/fastdds/**/*.h*")
set(INCLUDE_FILE "${PROJECT_BINARY_DIR}/external/eprosima/src/fastrtps/include/fastdds")

# Configure doxygen
configure_file(doxygen-config.in ${PROJECT_BINARY_DIR}/doxygen-config @ONLY)
add_custom_command(OUTPUT "${PROJECT_BINARY_DIR}/doxygen/xml/index.xml"
    COMMAND "${DOXYGEN_EXECUTABLE}" "${PROJECT_BINARY_DIR}/doxygen-config"
    DEPENDS ${HPP_FILES}
    )
# Generate API reference
add_custom_target(doxygen ALL
    DEPENDS "${PROJECT_BINARY_DIR}/doxygen/xml/index.xml"
    COMMENT "Generated API documentation with doxygen" VERBATIM)
add_dependencies(doxygen doc-dirs)

########################################################################################################################
# Build example and tests
########################################################################################################################
if(BUILDCOP)
    set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${PROJECT_BINARY_DIR}/external/eprosima/install")
    find_package(fastrtps REQUIRED)

    ####################################################################################################################
    # Compile example
    ####################################################################################################################
    add_executable(doctest CodeTester.cpp)
    target_include_directories(doctest PRIVATE ${PROJECT_BINARY_DIR}/external/eprosima/src/fastrtps/src/cpp
        ${PROJECT_BINARY_DIR}/external/eprosima/src/fastrtps/thirdparty/tinyxml2)
    target_link_libraries(doctest fastrtps)

    ####################################################################################################################
    # Add tests
    ####################################################################################################################
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
    set_property(TEST xmltest
        APPEND PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}/external/eprosima/install/lib")

    add_test(NAME governancetest COMMAND doctest GovernanceTester.xml)
    set_property(TEST governancetest
        APPEND PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}/external/eprosima/install/lib")

    add_test(NAME permissionstest COMMAND doctest PermissionsTester.xml)
    set_property(TEST permissionstest
        APPEND PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}/external/eprosima/install/lib")

    add_test(NAME statictest COMMAND doctest StaticTester.xml)
    set_property(TEST statictest
        APPEND PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}/external/eprosima/install/lib")
endif()
