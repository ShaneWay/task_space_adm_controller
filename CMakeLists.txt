# ----------------------------------------------------------------------------
#  CMakeLists.txt for C++ Kortex API Examples
#  Copyright © 2018 Kinova Robotics
#
#  Build instructions for the supported platforms (CMake only) : 
#
#  In Linux (x86-64 architecture):
#  From the api_cpp/examples directory, invoke:
#    $ mkdir build
#    $ cd build/
#    $ cmake .. 
#    $ make
#
#  In Windows (MinGW) :
#  From the api_cpp/examples directory, invoke:
#    $ mkdir build
#    $ cd build/
#    $ cmake .. -G "MinGW Makefiles"
#    $ mingw32-make
#
#  In Windows (MSVC command line) :
#  From the api_cpp/examples directory, invoke:
#    $ call "C:\Program Files (x86)\Microsoft Visual Studio\2017\BuildTools\VC\Auxiliary\Build\vcvars64.bat" [MODIFY THIS PATH FOR YOURS]
#    $ mkdir build
#    $ cd build/
#    $ cmake .. -G "NMake Makefiles"
#    $ nmake
#
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.20)

project(task_space_adm_control VERSION 2.6.0 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR   ON)
# set(CMAKE_VERBOSE_MAKEFILE      ON)
set(CMAKE_COLOR_MAKEFILE        ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

# Activate C++ 11
set (CMAKE_CXX_STANDARD 17)

include_directories(${PROJECT_SOURCE_DIR}/thirdParty/cxxopts/)

add_definitions(-D_OS_UNIX)

# link other libs
link_libraries(pthread)
link_libraries(${PROJECT_SOURCE_DIR}/lib/kortex/libKortexApiCpp.a)


find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
find_package(Eigen3)
find_package(yaml-cpp)
find_package(pinocchio REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/utility)
include_directories(${PROJECT_SOURCE_DIR}/controller)
include_directories(${PROJECT_SOURCE_DIR}/force_sensor)
include_directories(${PROJECT_SOURCE_DIR}/dynamics_model)
include_directories(${PROJECT_SOURCE_DIR}/kinematics_model)
include_directories(${PROJECT_SOURCE_DIR}/thirdParty/cxxopts)
include_directories(${PROJECT_SOURCE_DIR}/Parameters)
include_directories(${PROJECT_SOURCE_DIR}/include/kortex)
include_directories(${PROJECT_SOURCE_DIR}/include/kortex/client)
include_directories(${PROJECT_SOURCE_DIR}/include/kortex/common)
include_directories(${PROJECT_SOURCE_DIR}/include/kortex/messages)
include_directories(${PROJECT_SOURCE_DIR}/include/kortex/client_stubs)
include_directories(${PROJECT_SOURCE_DIR}/include/kortex/message)
# include_directories(${PROJECT_SOURCE_DIR}/include/kortex/thirdParty/include)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIR})

message(${PROJECT_SOURCE_DIR})
# Create executable for each example
# Look for examples under folders
file(GLOB_RECURSE EXE_LIST RELATIVE ${PROJECT_SOURCE_DIR}/src "[0-9]*.cpp")
MESSAGE(${EXE_LIST})
foreach ( SRC_FILE ${EXE_LIST} )

  string(REPLACE ".cpp" "" TARGET_EXE_NAME ${SRC_FILE})
  string(REPLACE "/" "_" TARGET_EXE_NAME ${TARGET_EXE_NAME})
  MESSAGE(${SRC_FILE})
  MESSAGE("creating TARGET_EXE_NAME: '${TARGET_EXE_NAME}'")

  add_executable(${TARGET_EXE_NAME} src/${SRC_FILE} 
                    ./utility/utilities.cpp
                    )

  if(TARGET_EXE_NAME MATCHES "^01")
      target_sources(${TARGET_EXE_NAME} PRIVATE
          ./controller/Controller.cpp
          ./controller/ControlLogger.cpp
      )
  endif()

  target_include_directories(${TARGET_EXE_NAME} PUBLIC ${Python3_INCLUDE_DIRS})
  target_include_directories(${TARGET_EXE_NAME} PUBLIC ${Python3_NumPy_INCLUDE_DIRS})

  target_link_libraries(${TARGET_EXE_NAME} PUBLIC
                  Python3::Python
                  Python3::Module
                  yaml-cpp
                  pinocchio::pinocchio # Linked Pinocchio here
                  Eigen3::Eigen        # Modern CMake linking for Eigen
                  stdc++fs
                  pthread
  )   

    if(Python3_NumPy_FOUND)
        target_link_libraries(${TARGET_EXE_NAME} PUBLIC
        Python3::NumPy
      )
    message("have numpy")
    message(${Python3_NumPy_FOUND})
    else()
        target_compile_definitions(${TARGET_EXE_NAME} INTERFACE WITHOUT_NUMPY)
        message("no numpy")
    endif()

endforeach()

