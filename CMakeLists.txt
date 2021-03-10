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

cmake_minimum_required(VERSION 3.12)

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


#########################################################################################################################
# Build Doxygen documentation
#########################################################################################################################

find_package(fastrtps REQUIRED)
get_target_property(FAST_INCLUDE_DIR fastrtps INTERFACE_INCLUDE_DIRECTORIES)

message(STATUS "Fast DDS include directories: ${FAST_INCLUDE_DIR}")
file(GLOB_RECURSE HPP_FILES "${FAST_INCLUDE_DIR}/fastdds/**/*.h*")

option(BUILD_DOCUMENTATION "Generate documentation" ON)

if (BUILD_DOCUMENTATION)
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

    ########################################################################################################################
    # Build Sphinx documentation
    ########################################################################################################################
    set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/code/cmake" ${CMAKE_MODULE_PATH})
    find_package(Sphinx REQUIRED)

    set(SPHINX_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/docs")
    set(SPHINX_BUILD "${PROJECT_BINARY_DIR}/html")

    add_custom_target(Sphinx ALL
                    COMMAND
                    ${SPHINX_EXECUTABLE} -b html
                    # Tell Breathe where to find the Doxygen output
                    -Dbreathe_projects.FastDDS=${DOXYGEN_OUTPUT_DIR}/xml
                    -d "${PROJECT_BINARY_DIR}/doctrees" ${SPHINX_SOURCE} ${SPHINX_BUILD}
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                    DEPENDS doxygen
                    COMMENT "Generating documentation with Sphinx")
endif()

option(COMPILE_TESTS "Compile Fast DDS-docs tests" ON)

if(COMPILE_TESTS)
    add_subdirectory(code)
endif()
