# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.14)

## MAIN project
project(CentroidalMPCWalking
  VERSION 0.0.1)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

option(BUILD_TESTING "Create tests using CMake" OFF)
include(CTest)

option(BUILD_DEVICE_EXAMPLES "Create example devices using CMake" OFF)

#set default build type to "Release" in single-config generators
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, recommanded options are: Debug or Release" FORCE)
    endif()
    set(ROBOTOLOGY_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${ROBOTOLOGY_BUILD_TYPES})
endif()

# Defines the CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR and many other useful macros.
# See https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
# With the following settings executables are placed in <the top level of the
# build tree>/bin and libraries/archives in <top level of the build tree>/lib.
# This is particularly useful to run ctests on libraries built on Windows
# machines: tests, which are executables, are placed in the same folders of
# dlls, which are treated as executables as well, so that they can properly
# find the libraries to run. This is a because of missing RPATH on Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE.
# See https://cmake.org/cmake/help/v3.4/variable/CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS.html
# See https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Under MSVC, we set CMAKE_DEBUG_POSTFIX to "d" to add a trailing "d" to library
# built in debug mode. In this Windows user can compile, build and install the
# library in both Release and Debug configuration avoiding naming clashes in the
# installation directories.
if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)


find_package(Eigen3 3.2.92 REQUIRED)
find_package(BipedalLocomotionFramework 0.3.100 REQUIRED
  COMPONENTS ParametersHandlerYarpImplementation RobotInterfaceYarpImplementation
             FloatingBaseEstimators IK ContinuousDynamicalSystem System
             SystemYarpImplementation ReducedModelControllers Planners ManifConversions
             ContactDetectors SimplifiedModelControllers)

#Function to automatize the process of creating a new library
include(AddCentroidalMPCWalkingApplication)
include(InstallIniFiles)

add_subdirectory(src)
add_subdirectory(worlds)

# include(InstallBasicPackageFiles)
# set(PROJECT_PRIVATE_DEPENDENCIES )
# set(PROJECT_PUBLIC_DEPENDENCIES BipedalLocomotionFramework)
# install_basic_package_files(${PROJECT_NAME}
#                             NAMESPACE CentroidalMPCWalking::
#                             VERSION ${CentroidalMPCWalking_VERSION}
#                             COMPATIBILITY AnyNewerVersion
#                             TARGETS_PROPERTY CentroidalMPCWalking_TARGETS
#                             VARS_PREFIX ${PROJECT_NAME}
#                             NO_CHECK_REQUIRED_COMPONENTS_MACRO
#                             DEPENDENCIES ${PROJECT_PUBLIC_DEPENDENCIES}
#                             PRIVATE_DEPENDENCIES ${PROJECT_PRIVATE_DEPENDENCIES}
#                             OVERRIDE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake)


include(AddUninstallTarget)
