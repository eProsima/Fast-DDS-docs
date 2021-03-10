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

####################################################################################################################
# Generate smime files from security XMLs
####################################################################################################################

find_program(OPENSSL openssl DOC "path to OpenSSL executable")
message(STATUS "OpenSSL found: ${OPENSSL}")
if(OPENSSL)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/certs)

    execute_process(COMMAND ${OPENSSL} smime
        -sign -in ${CMAKE_CURRENT_SOURCE_DIR}/GovernanceTester.xml
        -text -out ${CMAKE_CURRENT_BINARY_DIR}/certs/GovernanceTester.smime
        -signer ${CMAKE_CURRENT_SOURCE_DIR}/certs/maincacert.pem
        -inkey ${CMAKE_CURRENT_SOURCE_DIR}/certs/maincakey.pem)

        execute_process(COMMAND ${OPENSSL} smime
        -sign -in ${CMAKE_CURRENT_SOURCE_DIR}/PermissionsTester.xml
        -text -out ${CMAKE_CURRENT_BINARY_DIR}/certs/PermissionsTester.smime
        -signer ${CMAKE_CURRENT_SOURCE_DIR}/certs/maincacert.pem
        -inkey ${CMAKE_CURRENT_SOURCE_DIR}/certs/maincakey.pem)
else()
    message(FATAL_ERROR "Could not find OpenSSL")
endif()

####################################################################################################################
# Compile getting started tutorial
####################################################################################################################
add_subdirectory(Examples/C++/DDSHelloWorld)

####################################################################################################################
# Compile example
####################################################################################################################

add_executable(doctest CodeTester.cpp)
target_include_directories(doctest PRIVATE ${FAST_INCLUDE_DIR})
target_link_libraries(doctest fastrtps)

add_executable(dds-doctest DDSCodeTester.cpp FastDDSGenCodeTester.cpp)
target_include_directories(dds-doctest PRIVATE ${FAST_INCLUDE_DIR})
target_link_libraries(dds-doctest fastrtps)

####################################################################################################################
# Add tests
####################################################################################################################
enable_testing()
include(CTest)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/XMLTester.xml
    ${CMAKE_CURRENT_BINARY_DIR}/XMLTester.xml COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/XMLTesterExample.xml
    ${CMAKE_CURRENT_BINARY_DIR}/XMLTesterExample.xml COPYONLY)

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

add_test(NAME xmltest COMMAND doctest XMLTester.xml)

add_test(NAME xmltestexample COMMAND doctest XMLTesterExample.xml)

add_test(NAME permissions_test
    COMMAND doctest
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/maincacert.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/appcert.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/appkey.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/GovernanceTester.smime"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/PermissionsTester.smime")

add_test(NAME dds_permissions_test
    COMMAND dds-doctest
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/maincacert.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/appcert.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/appkey.pem"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/GovernanceTester.smime"
    "file://${CMAKE_CURRENT_BINARY_DIR}/certs/PermissionsTester.smime")


add_test(NAME statictest COMMAND doctest StaticTester.xml)

find_package(Python COMPONENTS Interpreter Development)
if(${Python_FOUND} AND (${Python_VERSION} VERSION_GREATER "3.6"))
    add_test(NAME helloworld COMMAND ${Python_EXECUTABLE} helloworld_test.py)
    set_property(TEST helloworld
        APPEND PROPERTY ENVIRONMENT "HELLOWORLD_PUB_TEST_BIN=$<TARGET_FILE:DDSHelloWorldPublisher>")
    set_property(TEST helloworld
        APPEND PROPERTY ENVIRONMENT "HELLOWORLD_SUB_TEST_BIN=$<TARGET_FILE:DDSHelloWorldSubscriber>")
endif()
