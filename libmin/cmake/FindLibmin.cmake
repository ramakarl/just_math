
#
# Find LibHelp
#

unset(LIBMIN_FOUND CACHE)
unset(LIBMIN_INC_DIR CACHE)

# LIBMIN_ROOT_DIR - build output directory for libmin

if ( NOT DEFINED LIBMIN_ROOT_DIR )
  if (WIN32)
    get_filename_component ( BASEDIR "${CMAKE_CURRENT_BINARY_DIR}/../libmin" REALPATH )
  else()
    get_filename_component ( BASEDIR "/usr/local/libmin/" REALPATH )
  endif()
  set ( LIBMIN_ROOT_DIR ${BASEDIR} CACHE PATH "Location of libmin" FORCE)
endif()
message ( STATUS "Searching for libhelp at.. ${LIBMIN_ROOT_DIR}")
set( LIBMIN_FOUND "YES" )

if ( LIBMIN_ROOT_DIR )

    #-- Paths
	set ( LIBMIN_INC_DIR "${LIBMIN_ROOT_DIR}/include" CACHE PATH "Path to include files" FORCE)
    set ( LIBMIN_LIB_DIR "${LIBMIN_ROOT_DIR}/bin" CACHE PATH "Path to libraries" FORCE)
	set ( LIBMIN_GLEW_DIR "${LIBMIN_INC_DIR}/GL" CACHE PATH "Path to glew.c" FORCE)

	#-------- Locate Header files
    set ( OK_H "0" )
	_FIND_FILE ( LIBMIN_FILES LIBMIN_INC_DIR "common_defs.h" "common_defs.h" OK_H )
	_FIND_FILE ( LIBMIN_FILES LIBMIN_INC_DIR "nv_gui.h" "nv_gui.h" OK_H )
	_FIND_FILE ( LIBMIN_FILES LIBMIN_INC_DIR "vec.h" "vec.h" OK_H )
	_FIND_FILE ( LIBMIN_FILES LIBMIN_INC_DIR "timex.h" "timex.h" OK_H )
	if ( OK_H EQUAL 4 )
	    message ( STATUS "  Found. Libhelp header files. ${LIBMIN_INCLUDE_DIR}" )
	else()
	    message ( "  NOT FOUND. Libhelp Header files" )
  	    set ( LIBMIN_FOUND "NO" )
	endif ()

    #-------- Locate Library
    set ( LIST_DLL "" )
	set ( LIST_DEBUG "" )
	set ( LIST_REL "" )
    set ( OK_DLL 0 )
    set ( OK_LIB 0 )
	_FIND_FILE ( LIST_DEBUG LIBMIN_LIB_DIR "libmind.lib" "libmind.so" OK_LIB )
	_FIND_FILE ( LIST_DLL LIBMIN_LIB_DIR "libmind.dll" "libmind.so" OK_DLL )
  
	_FIND_FILE ( LIST_REL LIBMIN_LIB_DIR "libmin.lib" "libmin.so" OK_LIB )
	_FIND_FILE ( LIST_DLL LIBMIN_LIB_DIR "libmin.dll" "libmin.so" OK_DLL )

	if ( (${OK_DLL} GREATER_EQUAL 1) AND (${OK_LIB} GREATER_EQUAL 1) )
	   message ( STATUS "  Found. Libhelp Library. ${LIBMIN_LIB_DIR}" )
	else()
	   set ( LIBMIN_FOUND "NO" )
	   message ( "  NOT FOUND. Libhelp Library. (so/dll or lib missing)" )
	endif()

endif()

if ( ${LIBMIN_FOUND} STREQUAL "NO" )
   message( FATAL_ERROR "
      Please set LIBMIN_ROOT_DIR to the root location
      of installed Libhelp library containing LIBMIN_full.lib/dll
      Not found at LIBMIN_ROOT_DIR: ${LIBMIN_ROOT_DIR}\n"
   )
endif()

set ( LIBMIN_DLLS ${LIST_DLL} CACHE INTERNAL "" FORCE)
set ( LIBMIN_DEBUG ${LIST_DEBUG} CACHE INTERNAL "" FORCE)
set ( LIBMIN_REL ${LIST_REL} CACHE INTERNAL "" FORCE)

#-- We do not want user to modified these vars, but helpful to show them
message ( STATUS "  LIBMIN_ROOT_DIR: ${LIBMIN_ROOT_DIR}" )
message ( STATUS "  LIBMIN_LIB_DIR:  ${LIBMIN_LIB_DIR}" )
message ( STATUS "  LIBMIN_INC_DIR:  ${LIBMIN_INC_DIR}" )
message ( STATUS "  LIBMIN_GLEW_DIR: ${LIBMIN_GLEW_DIR}" )
message ( STATUS "  LIBMIN_DLL:      ${LIBMIN_DLLS}" )
message ( STATUS "  LIBMIN_DEBUG:    ${LIBMIN_DEBUG}" )
message ( STATUS "  LIBMIN_REL:      ${LIBMIN_REL}" )

mark_as_advanced(LIBMIN_FOUND)


