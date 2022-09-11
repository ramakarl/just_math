cmake_minimum_required(VERSION 2.8)
set(PROJNAME math_deform)
Project(${PROJNAME})
Message(STATUS "-------------------------------")
Message(STATUS "Processing Project ${PROJNAME}:")

#####################################################################################
# Bootstrap
#
set( BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
find_path ( CMAKE_HELPERS_PATH "HelpersBootstrap.cmake" HINTS ${BASE_DIRECTORY}/../libmin/cmake/ )
if ( ${CMAKE_HELPERS_PATH} STREQUAL "HELPERS-NOTFOUND" )
    message ( FATAL_ERROR "\n Please set the CMAKE_HELPERS_PATH to location of HelpersBootstrap.cmake" )
endif()
include( ${CMAKE_HELPERS_PATH}/HelpersBootstrap.cmake )		# Cross-Platform functions
include_directories ("${CMAKE_CURRENT_SOURCE_DIR}/src")
include_directories ("${CMAKE_CURRENT_SOURCE_DIR}/include")

#####################################################################################
# Options

_REQUIRE_MAIN()

#--- symbols in release mode
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG /OPT:REF /OPT:ICF" CACHE STRING "" FORCE)


#####################################################################################
# Include LIBMIN
#
find_package(Libmin)

if (LIBMIN_FOUND)
  add_definitions(-DUSE_LIBMIN)  
  include_directories(${LIBMIN_INC_DIR})
  include_directories(${LIBRARIES_INC_DIR})  
  LIST( APPEND LIBRARIES_OPTIMIZED "${LIBMIN_LIB_DIR}/${LIBMIN_REL}")
  LIST( APPEND LIBRARIES_DEBUG "${LIBMIN_LIB_DIR}/${LIBMIN_DEBUG}")	
  _EXPANDLIST( OUTPUT PACKAGE_DLLS SOURCE ${LIBMIN_LIB_DIR} FILES ${LIBMIN_DLLS} )
  message ( STATUS "  ---> Using LIBMIN")
endif()

###################################################################################
# OpenGL & GLEW
#
OPTION (BUILD_OPENGL "Build with OpenGL" ON)
if (BUILD_OPENGL)
    message ( STATUS "Searching for OpenGL and GLEW.." )
	find_package(OpenGL)
	add_definitions(-DUSE_OPENGL)  		# Use OpenGL
	IF (WIN32)
	     LIST(APPEND LIBRARIES_OPTIMIZED "opengl32.lib" )
	     LIST(APPEND LIBRARIES_DEBUG "opengl32.lib" )
	ENDIF()
	add_definitions(-DGLEW_STATIC)
	file(GLOB COMMON_SOURCE_FILES "${LIBMIN_SRC}/glew.c" )   # Use GLEW (static)
	message ( STATUS "  ---> Using GLEW: ${COMMON_SOURCE_FILES}" )
endif()



######################
# CMAKE_INSTALL_PREFIX -- path where library will be installed to

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
   if ( WIN32 )
      get_filename_component ( _instpath "${CMAKE_CURRENT_BINARY_DIR}" REALPATH )
   else()
      get_filename_component ( _instpath "/usr/local/shapes" REALPATH )
   endif()
   set ( CMAKE_INSTALL_PREFIX ${_instpath} CACHE PATH "default install path" FORCE)   
endif()

#####################################################################################
# Asset Path
#
if ( NOT DEFINED ASSET_PATH ) 
   get_filename_component ( _assets "${BASE_DIRECTORY}/assets" REALPATH )
   set ( ASSET_PATH ${_assets} CACHE PATH "Full path to /assets" )
   add_definitions( -DASSET_PATH="${ASSET_PATH}/" )
endif()
file(GLOB GLSL_FILES ${ASSET_PATH}/*.glsl )
add_definitions(-DASSET_PATH="${ASSET_PATH}/")

#####################################################################################
# Executable
#
file(GLOB MAIN_FILES *.cpp *.c *.h )

unset ( ALL_SOURCE_FILES )

list( APPEND ALL_SOURCE_FILES ${MAIN_FILES} )
list( APPEND ALL_SOURCE_FILES ${COMMON_SOURCE_FILES} )
list( APPEND ALL_SOURCE_FILES ${PACKAGE_SOURCE_FILES} )
list( APPEND ALL_SOURCE_FILES ${UTIL_SOURCE_FILES} )

if ( NOT DEFINED WIN32 )
    set(libdeps GL GLEW X11)
  LIST(APPEND LIBRARIES_OPTIMIZED ${libdeps})
  LIST(APPEND LIBRARIES_DEBUG ${libdeps})
ENDIF()
include_directories ("${CMAKE_CURRENT_SOURCE_DIR}")    

add_executable (${PROJNAME} ${ALL_SOURCE_FILES} ${GLSL_FILES} )

set_property ( TARGET ${PROJNAME} APPEND PROPERTY DEPENDS )

#--- debug and release exe
set ( CMAKE_DEBUG_POSTFIX "d" CACHE STRING "" )
set_target_properties( ${PROJNAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})

#####################################################################################
# Additional Libraries
#
_LINK ( PROJECT ${PROJNAME} OPT ${LIBRARIES_OPTIMIZED} DEBUG ${LIBRARIES_DEBUG} PLATFORM ${PLATFORM_LIBRARIES} )

#####################################################################################
# Windows specific
#
_MSVC_PROPERTIES()
source_group("Source Files" FILES ${MAIN_FILES} ${COMMON_SOURCE_FILES} ${PACKAGE_SOURCE_FILES})

#####################################################################################
# Install Binaries
#
#
_DEFAULT_INSTALL_PATH()

# *NOTE*: file COPY is at cmake-time, not compile-time. Need to replace with add_custom_command -E copy_directory (my own _COPY)

file (COPY "${CMAKE_CURRENT_SOURCE_DIR}/assets" DESTINATION ${EXECUTABLE_OUTPUT_PATH} )	# assets folder
_INSTALL ( FILES ${SHADERS} DESTINATION "${EXECUTABLE_OUTPUT_PATH}/assets" )		# shaders
_INSTALL ( FILES ${PACKAGE_DLLS} DESTINATION ${EXECUTABLE_OUTPUT_PATH} )		# DLLs
install ( FILES $<TARGET_PDB_FILE:${PROJNAME}> DESTINATION ${EXECUTABLE_OUTPUT_PATH} OPTIONAL )		# PDB

install ( FILES ${INSTALL_LIST} DESTINATION ${EXECUTABLE_OUTPUT_PATH} )		# exe, pdb

###########################
# Done
message ( STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}" )
message ( STATUS "CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}" )
message ( STATUS "EXECUTABLE_OUTPUT_PATH: ${EXECUTABLE_OUTPUT_PATH}" )




