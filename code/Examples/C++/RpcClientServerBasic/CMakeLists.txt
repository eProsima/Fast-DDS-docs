# Copyright 2025 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

project(RpcClientServerBasic)

# Find requirements
if(NOT fastcdr_FOUND)
    find_package(fastcdr 2 REQUIRED)
endif()

if(NOT fastdds_FOUND)
    find_package(fastdds 3.2.0 REQUIRED)
endif()

# Set C++11
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
    if(SUPPORTS_CXX11)
        add_compile_options(-std=c++11)
    else()
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()

message(STATUS "Configuring client server basic example...")
file(GLOB CLIENT_SERVER_BASIC_TYPES_SOURCES_CXX "src/types/*.cxx")
file(GLOB CLIENT_SERVER_BASIC_TYPES_SOURCES_IPP "src/types/*.ipp")

add_executable(basic_client src/CalculatorClient.cpp
    ${CLIENT_SERVER_BASIC_TYPES_SOURCES_CXX}
    ${CLIENT_SERVER_BASIC_TYPES_SOURCES_IPP}
)

target_link_libraries(basic_client fastdds fastcdr)

add_executable(basic_server src/CalculatorServer.cpp
    ${CLIENT_SERVER_BASIC_TYPES_SOURCES_CXX}
    ${CLIENT_SERVER_BASIC_TYPES_SOURCES_IPP}
)

target_link_libraries(basic_server fastdds fastcdr)
