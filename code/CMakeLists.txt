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

####################################################################################################
# Generate smime files from security XMLs
####################################################################################################

# OpenSSL is used to sign the security related XML configuration files. This is done in order to
# be able to check whether those XML files are correctly written. To do this, they are used to
# create a participant because this process expects the XML files to be signed, and consequently
# parses them.
find_program(OPENSSL openssl DOC "path to OpenSSL executable")
message(STATUS "OpenSSL found: ${OPENSSL}")
if(OPENSSL)
    # Create a directory for the signed files
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/certs)
    # Sign Governance file
    execute_process(COMMAND ${OPENSSL} smime
        -sign -in ${CMAKE_CURRENT_SOURCE_DIR}/GovernanceTester.xml
        -text -out ${CMAKE_CURRENT_BINARY_DIR}/certs/GovernanceTester.smime
        -signer ${CMAKE_CURRENT_SOURCE_DIR}/certs/maincacert.pem
        -inkey ${CMAKE_CURRENT_SOURCE_DIR}/certs/maincakey.pem)
    # Sign Persimissions file
    execute_process(COMMAND ${OPENSSL} smime
        -sign -in ${CMAKE_CURRENT_SOURCE_DIR}/PermissionsTester.xml
        -text -out ${CMAKE_CURRENT_BINARY_DIR}/certs/PermissionsTester.smime
        -signer ${CMAKE_CURRENT_SOURCE_DIR}/certs/maincacert.pem
        -inkey ${CMAKE_CURRENT_SOURCE_DIR}/certs/maincakey.pem)
else()
    message(FATAL_ERROR "Could not find OpenSSL")
endif()

####################################################################################################
# Compile getting started tutorial
####################################################################################################
add_subdirectory(Examples/C++/DDSHelloWorld)

####################################################################################################
# Compile test applications
####################################################################################################

# Compile all the code used in CPP snippets and link it to Fast DDS. This way, we can assure that
# the snippets code is at least valid from a compilation point of view.

# CodeTester utilizes Fast RTPS API
add_executable(doctest CodeTester.cpp)
target_include_directories(doctest PRIVATE ${FAST_INCLUDE_DIR})
target_link_libraries(doctest fastrtps)
if(MSVC)
    target_compile_definitions(doctest PRIVATE _CRT_SECURE_NO_WARNINGS)
endif(MSVC)

# DDSCodeTester utilizes Fast DDS API
add_executable(dds-doctest DDSCodeTester.cpp FastDDSGenCodeTester.cpp)
target_include_directories(dds-doctest PRIVATE ${FAST_INCLUDE_DIR})
target_link_libraries(dds-doctest fastrtps fastcdr)

####################################################################################################
# Add tests
####################################################################################################

if(WIN32)
# Use fastdds.bat for cli testing
# cmd can no longer be used because ctest resolves all commands binaries to forward slashes, see:
# https://gitlab.kitware.com/cmake/cmake/-/issues/17067
set(SHELL_INVOKE powershell -C)
endif(WIN32)

# Copy the neccessary files over to the build directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/XMLTester.xml
    ${CMAKE_CURRENT_BINARY_DIR}/XMLTester.xml COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/XMLTesterExample.xml
    ${CMAKE_CURRENT_BINARY_DIR}/XMLTesterExample.xml COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/XMLTesterSkipValidation.xml
    ${CMAKE_CURRENT_BINARY_DIR}/XMLTesterSkipValidation.xml COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/certs/maincacert.pem
    ${CMAKE_CURRENT_BINARY_DIR}/certs/maincacert.pem COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/certs/appcert.pem
    ${CMAKE_CURRENT_BINARY_DIR}/certs/appcert.pem COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/certs/appkey.pem
    ${CMAKE_CURRENT_BINARY_DIR}/certs/appkey.pem COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/GovernanceTester.xml
    ${CMAKE_CURRENT_BINARY_DIR}/GovernanceTester.xml COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/PermissionsTester.xml
    ${CMAKE_CURRENT_BINARY_DIR}/PermissionsTester.xml COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/StaticTester.xml
    ${CMAKE_CURRENT_BINARY_DIR}/StaticTester.xml COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/helloworld_test.py
    ${CMAKE_CURRENT_BINARY_DIR}/helloworld_test.py COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/grep.py
    ${CMAKE_CURRENT_BINARY_DIR}/grep.py COPYONLY)

configure_file(${FAST_INCLUDE_DIR}/../share/fastRTPS_profiles.xsd
    ${CMAKE_CURRENT_BINARY_DIR}/fastRTPS_profiles.xsd COPYONLY)

# Check validity of Permissions and Governance XML files using Fast RTPS API
add_test(NAME xml.fastrtps_permissions_test
    COMMAND doctest
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/maincacert.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/appcert.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/appkey.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/GovernanceTester.smime"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/PermissionsTester.smime")

# Check validity of XML profiles snippets
add_test(NAME xml.profiles COMMAND dds-doctest XMLTester.xml)
# Check validity of XML full example
add_test(NAME xml.example COMMAND dds-doctest XMLTesterExample.xml)
# Validate XML full example against XSD schema
add_test(NAME xml.validate COMMAND ${SHELL_INVOKE} fastdds xml validate XMLTesterExample.xml -x fastRTPS_profiles.xsd)
# Validate XML snippets doc against XSD schema
add_test(NAME xml.snippets COMMAND ${SHELL_INVOKE} fastdds xml validate XMLTester.xml -x fastRTPS_profiles.xsd)
# Check validity of Permissions and Governance XML files using Fast DDS API
add_test(NAME xml.fastdds_permissions_test
    COMMAND dds-doctest
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/maincacert.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/appcert.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/appkey.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/GovernanceTester.smime"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/PermissionsTester.smime")
# Check validity of Static discovery XML file
add_test(NAME xml.static_discovery COMMAND dds-doctest StaticTester.xml)

# Check that the helloworld example runs as expected
find_package(Python3 COMPONENTS Interpreter Development)
if(${Python3_FOUND} AND (${Python3_VERSION} VERSION_GREATER "3.6"))
    add_test(NAME helloworld COMMAND ${Python3_EXECUTABLE} helloworld_test.py)
    set_property(TEST helloworld
        APPEND PROPERTY ENVIRONMENT "HELLOWORLD_PUB_TEST_BIN=$<TARGET_FILE:DDSHelloWorldPublisher>")
    set_property(TEST helloworld
        APPEND PROPERTY ENVIRONMENT "HELLOWORLD_SUB_TEST_BIN=$<TARGET_FILE:DDSHelloWorldSubscriber>")
endif()

# Check docs style
find_program(DOC8 doc8 DOC "path to doc8 executable")
message(STATUS "doc8 found: ${DOC8}")
if(DOC8)
    add_test(NAME doc8-test COMMAND ${DOC8} --max-line-length 120 ${CMAKE_SOURCE_DIR}/docs)
else()
    message(FATAL_ERROR "Could not find doc8")
endif()

# Check that all cpp snippets are on the CPP files and not in the RST files.
# This way, we are sure that the C++ is compiled
add_test(NAME docs.code_block_check.cpp
    COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/grep.py
    --directory ${CMAKE_SOURCE_DIR}/docs
    --regex "code-block::.*c[+p][+p]")

# Check that all XML snippets are on the XML files and not in the RST files.
# This way, we are sure that they are tested
add_test(NAME docs.code_block_check.xml
    COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/grep.py
    --directory ${CMAKE_SOURCE_DIR}/docs
    --regex "code-block::.*xml")

# Check docs spelling
add_test(NAME docs.spell_check
    COMMAND
    ${SPHINX_EXECUTABLE} -Q -W --keep-going
    -D breathe_projects.FastDDS=${DOXYGEN_OUTPUT_DIR}/xml
    -b spelling
    -d "${PROJECT_BINARY_DIR}/doctrees"
    ${CMAKE_SOURCE_DIR}/docs
    ${CMAKE_BINARY_DIR}/spelling)

# Populate environment variables for tests
if(WIN32)
    get_directory_property(DOCS_TESTS TESTS)
    set_tests_properties(${DOCS_TESTS} PROPERTIES
        ENVIRONMENT_MODIFICATION "PATH=path_list_append:$<TARGET_FILE_DIR:fastrtps>"
        ENVIRONMENT_MODIFICATION "PATH=path_list_append:$<TARGET_FILE_DIR:fastcdr>"
    )
endif(WIN32)
