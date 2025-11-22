cmake_minimum_required(VERSION 2.8...3.5)

set(PROJNAME math_gcoder )
Project(${PROJNAME} LANGUAGES C CXX)

Message(STATUS "-------------------------------")
Message(STATUS "Project: ${PROJNAME}")

#####################################################################################
# LIBMIN Bootstrap
#
set ( LIBMIN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../libmin" CACHE STRING "Location of Libmin")
list( APPEND CMAKE_MODULE_PATH "${LIBMIN_ROOT}/cmake" )
list( APPEND CMAKE_PREFIX_PATH "${LIBMIN_ROOT}/cmake" )

find_package(Libmin)

#####################################################################################
# Options

_REQUIRE_DATAPTR()
_REQUIRE_IMAGE()
_REQUIRE_3D()
_REQUIRE_GXLIB()

_REQUIRE_MAIN()

_REQUIRE_GL()

_REQUIRE_GLEW()

#####################################################################################
# Executable
#
file(GLOB MAIN_FILES *.cpp *.c *.h )

_GET_GLOBALS()

add_executable (${PROJNAME} ${MAIN_FILES} ${CUDA_FILES} ${PACKAGE_SOURCE_FILES} ${LIBMIN_FILES})

_LINK ( PROJECT ${PROJNAME} OPT ${LIBS_OPTIMIZED} DEBUG ${LIBS_DEBUG} PLATFORM ${LIBS_PLATFORM} )

#####################################################################################
# IDE Setup
#
_STARTUP_PROJECT ()
source_group("Source Files" FILES ${MAIN_FILES} ${PACKAGE_SOURCE_FILES})
source_group("CUDA" FILES ${CUDA_FILES} )

#####################################################################################
# Install Binaries
#
install_files ( PROJ ${PROJNAME} FILES ${PACKAGE_DLLS} DESTINATION . )    	# DLLs
install ( FILES $<TARGET_PDB_FILE:${PROJNAME}> OPTIONAL DESTINATION .)   	# PDB
install ( FILES ${INSTALL_LIST} DESTINATION .)				# EXE

###########################
# Done
message ( STATUS "  Source Path: ${CMAKE_CURRENT_SOURCE_DIR}" )
message ( STATUS "  Binary Path: ${CMAKE_CURRENT_BINARY_DIR}" )
message ( STATUS "  Libs Optimized:   ${LIBS_OPTIMIZED}" )
message ( STATUS "  Libs Debug:  ${LIBS_DEBUG}" )
message ( STATUS "  Libs Platform:  ${LIBS_PLATFORM}" )
message ( STATUS "  Packaged DLLs:   ${PACKAGE_DLLS}" )
message ( STATUS "  OUTPUT PATH:     ${CMAKE_INSTALL_PREFIX}" )




