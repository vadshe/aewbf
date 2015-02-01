AC_INIT([aewbf],[0.01.0],[vadim@sigrand.com],[aewbf],[http://www.sigrand.com])

AC_PREREQ([2.69])
AM_INIT_AUTOMAKE([1.14 no-define])
AC_CONFIG_HEADERS([config.h])
#AC_CONFIG_FILES([Makefile])

#AC_SEARCH_LIBS([strerror],[cposix])
AC_PROG_CC
AM_PROG_CC_C_O
#AC_HEADER_STDC
#LT_INIT
AC_PROG_RANLIB
LT_INIT

AC_ARG_ENABLE(debug,[--enable-debug  Enable debug notices],[debug=$enableval], [debug=no])

#GCC_FLAGS="-Wall"

if test "x${debug}" = "xyes"; then
  CFLAGS="-Wall -O0 -g"
else
  CFLAGS="-O2"
fi
AC_SUBST(CFLAGS)

dnl === WITHLIB_OPTION([opt_pfx], [outvar_pfx])
dnl ===   Defines --with-<opt_pfx>{include,lib}dir options which set
dnl ===   the variables <outvar_pfx>_{INCLUDES,LIBS}.
AC_DEFUN([WITHLIB_OPTION],
  [AC_ARG_WITH([$1includedir],
               AS_HELP_STRING([--with-$1includedir=DIR],
                              [use $2 includes from DIR]),
               $2_INCLUDES="-I$withval")
   AC_ARG_WITH([$1libdir],
               AS_HELP_STRING([--with-$1libdir=DIR],
                              [use $2 libraries from DIR]),
               [$2_LIBS="-L$withval"])])


SAVED_CPPFLAGS=$CPPFLAGS
SAVED_LIBS=$LIBS
CPPFLAGS="$PNG_INCLUDES $CPPFLAGS"
LIBS="$PNG_LIBS $LIBS"


CPPFLAGS=$SAVED_CPPFLAGS
LIBS=$SAVED_LIBS

AC_CONFIG_FILES([Makefile  src/Makefile])
AC_OUTPUT
