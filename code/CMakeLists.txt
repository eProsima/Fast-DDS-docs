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

project(fastdds-doc-checker VERSION "${LIB_VERSION_STR}" LANGUAGES C CXX)

########################################################################################################################
# Get fastdds git branch
########################################################################################################################
set(ENV_FASTDDS_BRANCH $ENV{FASTDDS_BRANCH})
set(FASTDDS_BRANCH "" CACHE STRING "Fast DDS repository branch against which test the source code.")

find_package(Git REQUIRED)

if(NOT "${ENV_FASTDDS_BRANCH}" STREQUAL "")
    set(REPO_TAG "${ENV_FASTDDS_BRANCH}")
elseif(NOT "${FASTDDS_BRANCH}" STREQUAL "")
    set(REPO_TAG "${FASTDDS_BRANCH}")
else()
    message("-- Environment variable FASTDDS_BRANCH not set. Setting fastdds branch to current documentation branch")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
        RESULT_VARIABLE RESULT
        OUTPUT_VARIABLE REPO_TAG)
endif()

# Remove trailing endline if present
string(REGEX REPLACE "\n$" "" REPO_TAG "${REPO_TAG}")

message("-- Checking whether fastdds branch ${REPO_TAG} exists...")
execute_process(
    COMMAND ${GIT_EXECUTABLE} ls-remote https://github.com/eProsima/Fast-DDS.git ${REPO_TAG}
    RESULT_VARIABLE RESULT_LS_REMOTE
    OUTPUT_VARIABLE OUTPUT_LS_REMOTE)

if("${OUTPUT_LS_REMOTE}" STREQUAL "")
    message("-- fastdds branch ${REPO_TAG} not found. Changing to master branch.")
    set(REPO_TAG "master")
endif()

########################################################################################################################
# Download and compile fastdds
########################################################################################################################

# Create directory fo external project
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/external)

# CMakeLists.txt for downloading and compiling Fast RTPS
set(PRODUCT_CMAKELIST_CONTENT "
    cmake_minimum_required(VERSION 3.0)
    project(fastdds-doc-external)
    include(ExternalProject)
    option(BUILDCOP \"Download buildcop.py\" OFF)
    set(BUILDCOP ${BUILDCOP})

    # Download Fast RTPS in same src directory used by buildcop
    ExternalProject_Add(fastdds
        PREFIX eprosima
        DOWNLOAD_DIR \"eprosima/src\"
        GIT_REPOSITORY https://github.com/eProsima/Fast-DDS.git
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
            DEPENDS fastdds
            )
    endif()
    ")
file(WRITE ${PROJECT_BINARY_DIR}/external/CMakeLists.txt "${PRODUCT_CMAKELIST_CONTENT}")

option(BUILDCOP "Activate actual fastdds building" OFF)
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
file(GLOB_RECURSE HPP_FILES "${PROJECT_BINARY_DIR}/external/eprosima/src/fastdds/include/fastdds/**/*.h*")

# Doxygen related variables
set(DOXYGEN_INPUT_DIR "${PROJECT_BINARY_DIR}/external/eprosima/src/fastdds/include/fastdds ${PROJECT_BINARY_DIR}/external/eprosima/src/fastdds/include/statistics")
set(DOXYGEN_OUTPUT_DIR "${PROJECT_BINARY_DIR}/doxygen")
set(DOXYGEN_INDEX_FILE "${PROJECT_BINARY_DIR}/doxygen/xml/index.xml")
set(DOXYFILE_IN "${CMAKE_CURRENT_SOURCE_DIR}/doxygen-config.in")
set(DOXYFILE_OUT ${PROJECT_BINARY_DIR}/doxygen-config)

# Configure doxygen
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

# Doxygen command
add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
    DEPENDS ${HPP_FILES}
    MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
    COMMENT "Generating doxygen documentation")

# Generate API reference
add_custom_target(doxygen ALL
    DEPENDS ${DOXYGEN_INDEX_FILE}
    COMMENT "Generated API documentation with doxygen" VERBATIM)
add_dependencies(doxygen doc-dirs)

########################################################################################################################
# Build Sphinx documentation
########################################################################################################################
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
find_package(Sphinx REQUIRED)

set(SPHINX_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../docs")
set(SPHINX_BUILD "${PROJECT_BINARY_DIR}/../html")

add_custom_target(Sphinx ALL
                  COMMAND
                  ${SPHINX_EXECUTABLE} -b html
                  # Tell Breathe where to find the Doxygen output
                  -Dbreathe_projects.FastDDS=${DOXYGEN_OUTPUT_DIR}/xml
                  -d "${PROJECT_BINARY_DIR}/../doctrees" ${SPHINX_SOURCE} ${SPHINX_BUILD}
                  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                  DEPENDS doxygen
                  COMMENT "Generating documentation with Sphinx")

########################################################################################################################
# Build example and tests
########################################################################################################################
if(BUILDCOP)
    set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${PROJECT_BINARY_DIR}/external/eprosima/install")
    find_package(fastrtps REQUIRED)

    ####################################################################################################################
    # Compile getting started tutorial
    ####################################################################################################################
    add_subdirectory(Examples/C++/DDSHelloWorld)

    ####################################################################################################################
    # Compile example
    ####################################################################################################################
    add_executable(doctest CodeTester.cpp DDSCodeTester.cpp FastDDSGenCodeTester.cpp)
    target_include_directories(doctest PRIVATE ${PROJECT_BINARY_DIR}/external/eprosima/src/fastdds/src/cpp
        ${PROJECT_BINARY_DIR}/external/eprosima/src/fastdds/thirdparty/tinyxml2)
    target_link_libraries(doctest fastrtps)

    ####################################################################################################################
    # Add tests
    ####################################################################################################################
    enable_testing()
    include(CTest)

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/XMLTester.xml
        ${CMAKE_CURRENT_BINARY_DIR}/XMLTester.xml COPYONLY)

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/XMLTesterExample.xml
        ${CMAKE_CURRENT_BINARY_DIR}/XMLTesterExample.xml COPYONLY)

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/GovernanceTester.xml
        ${CMAKE_CURRENT_BINARY_DIR}/GovernanceTester.xml COPYONLY)

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/PermissionsTester.xml
        ${CMAKE_CURRENT_BINARY_DIR}/PermissionsTester.xml COPYONLY)

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/StaticTester.xml
        ${CMAKE_CURRENT_BINARY_DIR}/StaticTester.xml COPYONLY)

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/helloworld_test.py
        ${CMAKE_CURRENT_BINARY_DIR}/helloworld_test.py COPYONLY)

    add_test(NAME xmltest COMMAND doctest XMLTester.xml)
    set_property(TEST xmltest
        APPEND PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}/external/eprosima/install/lib")

    add_test(NAME xmltestexample COMMAND doctest XMLTesterExample.xml)
        set_property(TEST xmltestexample
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

    find_package(Python COMPONENTS Interpreter Development)
    if(${Python_FOUND} AND (${Python_VERSION} VERSION_GREATER "3.6"))
        add_test(NAME helloworld COMMAND ${Python_EXECUTABLE} helloworld_test.py)
        set_property(TEST helloworld
            APPEND PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${PROJECT_BINARY_DIR}/external/eprosima/install/lib")
        set_property(TEST helloworld
            APPEND PROPERTY ENVIRONMENT "HELLOWORLD_PUB_TEST_BIN=$<TARGET_FILE:DDSHelloWorldPublisher>")
        set_property(TEST helloworld
            APPEND PROPERTY ENVIRONMENT "HELLOWORLD_SUB_TEST_BIN=$<TARGET_FILE:DDSHelloWorldSubscriber>")
    endif()
endif()
