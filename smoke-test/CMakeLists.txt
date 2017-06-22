# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.0)

# enforce C++11 features
set(CMAKE_CXX_STANDARD 11)

# set the project name
project(TestAssignmentSimpleControlDesign)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

# add the required cmake packages
find_package(RTF)
find_package(RTF 1.1 COMPONENTS DLL)
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

# add include directories
include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS})

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)

# add the source codes to build the plugin library
add_library(${PROJECT_NAME} MODULE test.cpp)

# add required libraries
target_link_libraries(${PROJECT_NAME} ${RTF_LIBRARIES} ${YARP_LIBRARIES} YARP::YARP_rtf ctrlLib)
