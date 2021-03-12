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


####################################################################################################
# Build Doxygen documentation
####################################################################################################

# Fast DDS is required for:
#    * Generating the doxygen XML files that are then use for the API reference
#    * Documentation tests applications
# It is required that Fast DDS is built with -DSECURITY=ON
find_package(fastrtps REQUIRED)
get_target_property(FAST_INCLUDE_DIR fastrtps INTERFACE_INCLUDE_DIRECTORIES)

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
# Build Sphinx documentation
####################################################################################################

# Find sphinx
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/code/cmake" ${CMAKE_MODULE_PATH})
find_package(Sphinx REQUIRED)

set(SPHINX_SOURCE "${CMAKE_SOURCE_DIR}/docs")

# It is possible to build and run tests without generating the documentation output. This saves time
# while developing, since generating the documentation from the RSTs takes quite some time.
option(BUILD_DOCUMENTATION "Generate documentation" ON)
if (BUILD_DOCUMENTATION)
    # Users can specify the sphinx builder with -DFASTDDS_DOCS_BUILDER. This option takes one value
    # of the lists FASTDDS_DOCS_ALLOWED_BUILDERS or TWO_STEP_BUILDERS. This second list is for
    # targets that require two steps in order to build the documentation. This way, the CMake poject
    # can cover all the options that the Makefile had. FASTDDS_DOCS_BUILDER defaults to html
    set(FASTDDS_DOCS_ALLOWED_BUILDERS
        html dirhtml singlehtml pickle json htmlhelp
        qthelp applehelp devhelp epub epub3 latex
        text man texinfo gettext changes xml pseudoxml pdf)
    set(TWO_STEP_BUILDERS latexpdf latexpdfja info)

    set(FASTDDS_DOCS_BUILDER html CACHE STRING "Select sphinx builder")
    set_property(
        CACHE FASTDDS_DOCS_BUILDER
        PROPERTY STRINGS "${FASTDDS_DOCS_ALLOWED_BUILDERS};${TWO_STEP_BUILDERS}")

    # Check that specified value is allowed
    message(STATUS "Selected builder: ${FASTDDS_DOCS_BUILDER}")
    if((NOT FASTDDS_DOCS_BUILDER IN_LIST FASTDDS_DOCS_ALLOWED_BUILDERS) AND
            (NOT FASTDDS_DOCS_BUILDER IN_LIST TWO_STEP_BUILDERS))
        message(FATAL_ERROR,
            "Wrong configuration of FASTDDS_DOCS_BUILDER.
            Allowed values: ${FASTDDS_DOCS_ALLOWED_BUILDERS};${TWO_STEP_BUILDERS}")
    endif()

    # Check whether a second step is required, and set option accordingly
    if("${FASTDDS_DOCS_BUILDER}" STREQUAL "latexpdf")
        set(INTERNAL_BUILDER "latex")
        set(SECOND_STEP_ARGS "all-pdf")
        set(DO_SECOND_STEP ON)
    elseif("${FASTDDS_DOCS_BUILDER}" STREQUAL "latexpdfja")
        set(INTERNAL_BUILDER "latex")
        set(SECOND_STEP_ARGS "all-pdf-ja")
        set(DO_SECOND_STEP ON)
    elseif("${FASTDDS_DOCS_BUILDER}" STREQUAL "info")
        set(INTERNAL_BUILDER "texinfo")
        set(SECOND_STEP_ARGS "info")
        set(DO_SECOND_STEP ON)
    else()
        set(INTERNAL_BUILDER ${FASTDDS_DOCS_BUILDER})
        set(DO_SECOND_STEP OFF)
    endif()

    # Generate the sphinx documentation
    add_custom_target(Sphinx ALL
        COMMAND
        ${SPHINX_EXECUTABLE} -b ${INTERNAL_BUILDER}
        # Tell Breathe where to find the Doxygen output
        -D breathe_projects.FastDDS=${DOXYGEN_OUTPUT_DIR}/xml
        -d "${PROJECT_BINARY_DIR}/doctrees"
        ${SPHINX_SOURCE}
        ${PROJECT_BINARY_DIR}/${INTERNAL_BUILDER}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS doxygen
        COMMENT "Generating documentation with Sphinx")

    # Do the second step if needed
    if(DO_SECOND_STEP)
        find_program(MAKE make DOC "path to make executable")
        message(STATUS "Make found: ${MAKE}")
        if(MAKE)
            add_custom_target(COMMAND ${MAKE}
                -C ${PROJECT_BINARY_DIR}/${INTERNAL_BUILDER}
                ${SECOND_STEP_ARGS})
        else()
            message(FATAL_ERROR "Could not find Make")
        endif()
    endif()

    # Install the generated docs
    install(DIRECTORY ${PROJECT_BINARY_DIR}/${INTERNAL_BUILDER}
        DESTINATION .
        COMPONENT ${INTERNAL_BUILDER}
        PATTERN ".buildinfo" EXCLUDE)
    set(CPACK_COMPONENT_EXAMPLES_DISPLAY_NAME "Fast DDS-docs ${INTERNAL_BUILDER}")
    set(CPACK_COMPONENT_EXAMPLES_DESCRIPTION
        "eProsima Fast DDS documetation in ${INTERNAL_BUILDER} format")
    set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} ${INTERNAL_BUILDER})
endif()

# Option to compile documentation tests
option(COMPILE_TESTS "Compile Fast DDS-docs tests" ON)
if(COMPILE_TESTS)
    # CTest needs to be included here, otherwise it is not possible to run the tests from the root
    # of the build directory
    enable_testing()
    include(CTest)
    add_subdirectory(code)
endif()
