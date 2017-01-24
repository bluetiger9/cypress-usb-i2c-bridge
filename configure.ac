#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
# Release version
m4_define([GENERIC_MAJOR_VERSION], [1])
m4_define([GENERIC_MINOR_VERSION], [1])
m4_define([GENERIC_MICRO_VERSION], [0])
m4_define([GENERIC_VERSION],
          m4_defn([GENERIC_MAJOR_VERSION]).m4_defn([GENERIC_MINOR_VERSION]).m4_defn([GENERIC_MICRO_VERSION]))

m4_define([GENERIC_RELEASE], m4_defn([GENERIC_MAJOR_VERSION]).m4_defn([GENERIC_MICRO_VERSION]))

# ------------------------------
# Initialization Macros
# ------------------------------
AC_PREREQ([2.63])
AC_INIT(cy3240_i2c, m4_defn([GENERIC_VERSION]), kevin.kirkup@sonyericsson.com)
AC_CONFIG_SRCDIR([src])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# ------------------------------
# Define version Package and version
# ------------------------------
LIBRARY_NAME=cy3240_i2c

AC_SUBST(LIBRARY_NAME)

# Library Version
LIBRARY_CURRENT_VERSION=1
LIBRARY_REVISION_VERSION=0
LIBRARY_AGE_VERSION=0

LIBRARY_VERSION=$LIBRARY_CURRENT_VERSION:$LIBRARY_REVISION_VERSION:$LIBRARY_AGE_VERSION
AC_SUBST(LIBRARY_VERSION)

RELEASE=$GENERIC_RELEASE
AC_SUBST(RELEASE)

# ------------------------------
# Initialize automake
# ------------------------------
AM_INIT_AUTOMAKE

# ------------------------------
# Checks for programs.
# ------------------------------
AC_PROG_CXX
AC_PROG_AWK
AC_PROG_CC
AC_PROG_CC_C99
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET

# ------------------------------
# Disable building of static libraries
# ------------------------------
AC_DISABLE_STATIC
AM_PROG_LIBTOOL

# ------------------------------
# Checks for libraries.
# ------------------------------
AC_CHECK_LIB([hidapi-libusb], [hid_open])
#AC_CHECK_LIB([usb], [libusb_alloc_transfer])

# ------------------------------
# Checks for header files.
# ------------------------------
AC_CHECK_HEADERS([stddef.h stdint.h stdlib.h string.h unistd.h])

# ------------------------------
# Checks for typedefs, structures, and compiler characteristics.
# ------------------------------
AC_HEADER_STDBOOL
AC_TYPE_INT16_T
AC_TYPE_INT32_T
AC_TYPE_INT8_T
AC_C_RESTRICT
AC_TYPE_SIZE_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T

# ------------------------------
# Checks for library functions.
# ------------------------------
AC_FUNC_MALLOC
AC_CHECK_FUNCS([memset])

# Custom Args
AC_ARG_ENABLE(debug,
              AS_HELP_STRING([--enable-debug], [Enable debug build support]),
              [build_debug=$enableval],
              [build_debug=no])

if test $build_debug = yes; then
     AC_DEFINE(DEBUG, 1, [1 to enable debug])
fi

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
