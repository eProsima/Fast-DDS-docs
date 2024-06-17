# Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

cmake_minimum_required(VERSION 3.20)

# Get version from sphinx's conf.py so that the cmake and the sphinx versions match
file(STRINGS
    ${CMAKE_SOURCE_DIR}/docs/conf.py
    LIB_VERSION_TMP
    REGEX "^version = u'[0-9]+.[0-9]+.[0-9]+'"
    )

string(REGEX REPLACE "^version = u'([0-9]+.[0-9]+.[0-9]+)'"
    "\\1"
    LIB_VERSION_STR
    ${LIB_VERSION_TMP}
    )

project(fastdds-docs VERSION "${LIB_VERSION_STR}" LANGUAGES C CXX)

# In case CMake version less than 3.24, add the option CMAKE_COMPILE_WARNING_AS_ERROR.
if (${CMAKE_VERSION} VERSION_LESS "3.24")
    option(CMAKE_COMPILE_WARNING_AS_ERROR "Treat warnings as errors" OFF)
endif()


####################################################################################################
# Build Doxygen documentation
####################################################################################################

# Fast DDS is required for:
#    * Generating the doxygen XML files that are then used for the API reference
#    * Documentation tests applications
# It is required that Fast DDS is built with -DSECURITY=ON
find_package(fastdds REQUIRED)
get_target_property(FAST_INCLUDE_DIR fastdds INTERFACE_INCLUDE_DIRECTORIES)

message(STATUS "Fast DDS include directories: ${FAST_INCLUDE_DIR}")
file(GLOB_RECURSE HPP_FILES "${FAST_INCLUDE_DIR}/fastdds/**/*.h*")

find_package(Doxygen REQUIRED)
# Create doxygen directories
add_custom_target(doc-dirs
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/doxygen
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/html
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/html/doxygen
    COMMENT "Creating documentation directories" VERBATIM)

# Doxygen related variables
set(DOXYGEN_INPUT_DIR "${FAST_INCLUDE_DIR}/fastdds")
set(DOXYGEN_OUTPUT_DIR "${PROJECT_BINARY_DIR}/doxygen")
set(DOXYGEN_INDEX_FILE "${PROJECT_BINARY_DIR}/doxygen/xml/index.xml")
set(DOXYFILE_IN "${CMAKE_CURRENT_SOURCE_DIR}/code/doxygen-config.in")
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

# Install doxygen generated XML files
install(DIRECTORY ${PROJECT_BINARY_DIR}/doxygen/xml
    DESTINATION doxygen
    COMPONENT doxygen)
set(CPACK_COMPONENT_EXAMPLES_DISPLAY_NAME "Fast DDS-docs doxygen")
set(CPACK_COMPONENT_EXAMPLES_DESCRIPTION
    "eProsima Fast DDS doxygen documetation")
set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} doxygen)

####################################################################################################
# Find Sphinx
####################################################################################################
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/code/cmake" ${CMAKE_MODULE_PATH})
find_package(Sphinx REQUIRED)

####################################################################################################
# Compile code used in documentation
####################################################################################################
# Option to compile documentation tests
option(COMPILE_TESTS "Compile Fast DDS-docs tests" ON)
if(COMPILE_TESTS)
    # CTest needs to be included here, otherwise it is not possible to run the tests from the root
    # of the build directory
    enable_testing()
    include(CTest)
    add_subdirectory(code)
endif()

####################################################################################################
# Build Sphinx documentation
####################################################################################################
# It is possible to build and run tests without generating the documentation output. This saves time
# while developing, since generating the documentation from the RSTs takes quite some time.
set(SPHINX_SOURCE "${CMAKE_SOURCE_DIR}/docs")
option(BUILD_DOCUMENTATION "Generate documentation" ON)
if (BUILD_DOCUMENTATION)
    # Check there is Python bindings accessible and configure sphinx depending on that.
    find_package(fastdds_python QUIET)
    set(PYTHON_EXTRA_FLAG "")
    if(NOT fastdds_python_FOUND)
        list(APPEND PYTHON_EXTRA_FLAG "-D;skip_python=")
    endif()

    # CMake project always build the documentaion using HTML builder. Users can always build with
    # other builders using sphinx directly
    set(FASTDDS_DOCS_BUILDER text)

    # Generate the sphinx documentation
    add_custom_target(Sphinx ALL
        COMMAND
        ${SPHINX_EXECUTABLE}
        $<$<BOOL:${CMAKE_COMPILE_WARNING_AS_ERROR}>:-W>
        -b ${FASTDDS_DOCS_BUILDER}
        # Tell Breathe where to find the Doxygen output
        -D breathe_projects.FastDDS=${DOXYGEN_OUTPUT_DIR}/xml
        ${PYTHON_EXTRA_FLAG}
        -d "${PROJECT_BINARY_DIR}/doctrees"
        ${SPHINX_SOURCE}
        ${PROJECT_BINARY_DIR}/${FASTDDS_DOCS_BUILDER}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS doxygen
        COMMENT "Generating documentation with Sphinx")

    # Install the generated docs
    install(DIRECTORY ${PROJECT_BINARY_DIR}/${FASTDDS_DOCS_BUILDER}
        DESTINATION .
        COMPONENT ${FASTDDS_DOCS_BUILDER}
        PATTERN ".buildinfo" EXCLUDE)
    set(CPACK_COMPONENT_EXAMPLES_DISPLAY_NAME "Fast DDS-docs ${FASTDDS_DOCS_BUILDER}")
    set(CPACK_COMPONENT_EXAMPLES_DESCRIPTION
        "eProsima Fast DDS documetation in ${FASTDDS_DOCS_BUILDER} format")
    set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} ${FASTDDS_DOCS_BUILDER})
endif()
