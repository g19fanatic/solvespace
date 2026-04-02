# Project-local wrapper for FindPkgConfig.cmake
# Fixes cmake 3.22 bug where PKG_CONFIG_VERSION is not set when find_package
# short-circuits due to cached PKG_CONFIG_EXECUTABLE
#
# See: https://gitlab.kitware.com/cmake/cmake/-/issues/24236

# Always ensure PKG_CONFIG_VERSION is set before the macro uses it
# Note: PKG_CONFIG_VERSION=1 is an INTERNAL version counter used by FindPkgConfig.cmake
# (not the pkg-config tool version). We use CACHE to persist it across cmake invocations.
set(PKG_CONFIG_VERSION 1 CACHE INTERNAL "FindPkgConfig internal version counter")

# Include the real system FindPkgConfig.cmake
# Use CMAKE_ROOT to find the system module location
include("${CMAKE_ROOT}/Modules/FindPkgConfig.cmake")
