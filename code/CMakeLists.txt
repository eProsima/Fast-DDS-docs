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
# Compile examples
####################################################################################################
add_subdirectory(Examples/C++/DDSHelloWorld)
add_subdirectory(Examples/C++/RpcClientServerBasic)
# Add Pro features only when Fast DDS Pro is being used
if(_fastdds_pro)
    add_subdirectory(Examples/C++/RpcClientServerFeed)
else()
    message(STATUS "Skipping RpcClientServerFeed example because Fast DDS Pro is not being used")
endif()

####################################################################################################
# Compile test applications
####################################################################################################

# Compile all the code used in CPP snippets and link it to Fast DDS. This way, we can assure that
# the snippets code is at least valid from a compilation point of view.

# CodeTester utilizes Fast RTPS API
add_executable(doctest CodeTester.cpp)
target_include_directories(doctest PRIVATE ${FAST_INCLUDE_DIR})
target_link_libraries(doctest fastdds fastcdr)

# DDSCodeTester utilizes Fast DDS API
set(DDS_CODE_TEST_SRC DDSCodeTester.cpp FastDDSGenCodeTester.cpp)
# Append Pro features only when Fast DDS Pro is being used
if(_fastdds_pro)
    list(APPEND DDS_CODE_TEST_SRC ProDDSCodeTester.cpp)
endif()
add_executable(dds-doctest ${DDS_CODE_TEST_SRC})
target_include_directories(dds-doctest PRIVATE ${FAST_INCLUDE_DIR})
target_link_libraries(dds-doctest fastdds fastcdr)

####################################################################################################
# Add tests
####################################################################################################

# Copy the neccessary files over to the build directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/XMLTester.xml
    ${CMAKE_CURRENT_BINARY_DIR}/XMLTester.xml COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/XMLTesterExample.xml
    ${CMAKE_CURRENT_BINARY_DIR}/XMLTesterExample.xml COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/XMLTesterSkipValidation.xml
    ${CMAKE_CURRENT_BINARY_DIR}/XMLTesterSkipValidation.xml COPYONLY)

if(_fastdds_pro)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ProXMLTester.xml
        ${CMAKE_CURRENT_BINARY_DIR}/ProXMLTester.xml COPYONLY)
endif()

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

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/rpc_calculator_basic_test.py
${CMAKE_CURRENT_BINARY_DIR}/rpc_calculator_basic_test.py COPYONLY)

if(_fastdds_pro)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/rpc_calculator_feed_test.py
  ${CMAKE_CURRENT_BINARY_DIR}/rpc_calculator_feed_test.py COPYONLY)
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/grep.py
    ${CMAKE_CURRENT_BINARY_DIR}/grep.py COPYONLY)

configure_file(${FAST_INCLUDE_DIR}/../share/fastdds/fastdds_profiles.xsd
    ${CMAKE_CURRENT_BINARY_DIR}/fastdds_profiles.xsd COPYONLY)

# Check validity of Permissions and Governance XML files using Fast RTPS API
add_test(NAME xml.rtps_permissions_test
    COMMAND doctest
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/maincacert.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/appcert.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/appkey.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/GovernanceTester.smime"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/PermissionsTester.smime")

# Check validity of XML profiles snippets
add_test(NAME xml.profiles COMMAND dds-doctest XMLTester.xml)
# Validate XML snippets doc against XSD schema
add_test(NAME xml.profiles.validate COMMAND fastdds xml validate XMLTester.xml -x fastdds_profiles.xsd)
# Check validity of XML full example
add_test(NAME xml.example COMMAND dds-doctest XMLTesterExample.xml)
# Validate XML full example against XSD schema
add_test(NAME xml.example.validate COMMAND fastdds xml validate XMLTesterExample.xml -x fastdds_profiles.xsd)
if(_fastdds_pro)
    # Check validity of XML pro features example
    add_test(NAME xml.pro COMMAND dds-doctest ProXMLTester.xml)
    # Validate XML pro features example against XSD schema
    add_test(NAME xml.pro.validate COMMAND fastdds xml validate ProXMLTester.xml -x fastdds_profiles.xsd)
endif()
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

# Check that the helloworld and RPC calculator examples run as expected
find_package(Python3 COMPONENTS Interpreter Development)
if(${Python3_FOUND} AND (${Python3_VERSION} VERSION_GREATER "3.6"))
    add_test(NAME helloworld COMMAND ${Python3_EXECUTABLE} helloworld_test.py)
    set_property(TEST helloworld
        APPEND PROPERTY ENVIRONMENT "HELLOWORLD_PUB_TEST_BIN=$<TARGET_FILE:DDSHelloWorldPublisher>")
    set_property(TEST helloworld
        APPEND PROPERTY ENVIRONMENT "HELLOWORLD_SUB_TEST_BIN=$<TARGET_FILE:DDSHelloWorldSubscriber>")

    add_test(NAME calculator_basic COMMAND ${Python3_EXECUTABLE} rpc_calculator_basic_test.py)
    set_property(TEST calculator_basic
        APPEND PROPERTY ENVIRONMENT "CLIENT_TEST_BIN=$<TARGET_FILE:basic_client>")
    set_property(TEST calculator_basic
        APPEND PROPERTY ENVIRONMENT "SERVER_TEST_BIN=$<TARGET_FILE:basic_server>")

    if(_fastdds_pro)
        add_test(NAME calculator_feed COMMAND ${Python3_EXECUTABLE} rpc_calculator_feed_test.py)
        set_property(TEST calculator_feed
            APPEND PROPERTY ENVIRONMENT "CLIENT_TEST_BIN=$<TARGET_FILE:feed_client>")
        set_property(TEST calculator_feed
            APPEND PROPERTY ENVIRONMENT "SERVER_TEST_BIN=$<TARGET_FILE:feed_server>")
    endif()
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
