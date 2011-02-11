dnl
dnl configure.ac --
dnl	process this file with autoconf to produce a configure script.
dnl

AC_INIT(pr2SoftMotion-genom, 1.0, openrobots@laas.fr)
AC_CONFIG_AUX_DIR(autoconf)

m4_include([libtool.m4])
m4_include([pkg.m4])
m4_include([robots.m4])
m4_sinclude([../acinclude.user.m4])

ABI_VERSION_INFO=
AC_SUBST(ABI_VERSION_INFO)
genServer="yes"
genTcl="yes"
genOpenprs="no"
genTclservClient="no"

AC_CANONICAL_HOST

dnl --- checks for programs ---------------------------------------------
dnl
LANG_CXX=yes
AC_SUBST(LANG_CXX)

AC_PROG_CC
if test "x$LANG_CXX" = "xyes"; then
    AC_PROG_CXX
fi

AC_PROG_INSTALL
AC_PROG_LIBTOOL
AC_SUBST(LIBTOOL_DEPS)

ROBOT_CFLAGS
test -z "$CFLAGS" && CFLAGS="${C_DEBUG_FLAGS} ${C_WARNING_FLAGS}"

ROBOT_PROG_MKDEP

dnl Checks for headers --------------------------------------------------
dnl
AC_CHECK_HEADERS(sys/cdefs.h)

AC_C_BIGENDIAN([ENDIANNESS=-DWORDS_BIGENDIAN], [ENDIANNESS=])
AC_SUBST(ENDIANNESS)

dnl --- Xenomai support -------------------------------------------------
dnl

OSAPI=posix
AC_ARG_WITH(xenomai,
	AC_HELP_STRING([--with-xenomai=yes|no], [configure to use Xenomai]),
	[opt_xenomai=$withval],[opt_xenomai=no])
AC_ARG_VAR([XENO_CONFIG], [Path to the xeno-config command for xenomai])

if test "x$opt_xenomai" = "xyes"; then
	AC_PATH_PROG([XENO_CONFIG], [xeno-config], [no],
		[/usr/xenomai/bin:${PATH}])
	if test "x$XENO_CONFIG" = "xno" ; then
		AC_MSG_FAILURE([--with-xenomai=yes specified, but xeno-config not found])
		opt_xenomai=no
	else
		AC_MSG_RESULT([Configuring to use Xenomai])
		OSAPI=xenomai
	fi
fi
AC_SUBST(OSAPI)

if test "$OSAPI" = "posix"; then
	AC_CHECK_PTHREADS
fi

dnl --- checks for functions --------------------------------------------
dnl

LIBS=
AC_SEARCH_LIBS([fabs], [m])
AC_SEARCH_LIBS([atan], [m])
AC_SUBST(LIBMATH, "${LIBS}")


dnl --- checks for libraries --------------------------------------------
dnl

dnl GenoM and pocolibs
PKG_CHECK_MODULES(GENOM, genom >= 2.6)
AC_SUBST(GENOM_CFLAGS)
AC_SUBST(GENOM_LIBS)

dnl Tcl
ifelse([yes], [yes], [ROBOT_LIB_TCL(/usr/lib/tcl8.5/)], [])

dnl transgen and oprs
ifelse([no], [yes], [
	PKG_CHECK_MODULES(TRANSGEN, transgen)
        AC_CONFIG_FILES([pr2SoftMotion-oprs.pc:autoconf/pr2SoftMotion-oprs.pc.in])
	], [])

dnl search libtclserv_client
ifelse([no], [yes], [
		PKG_CHECK_MODULES(TCLSERV_CLIENT_LIB, libtclserv_client)
		AC_CONFIG_FILES([pr2SoftMotion-tclserv_client.pc:autoconf/pr2SoftMotion-tclserv_client.pc.in])
		], [])

dnl --- Check for extra software ----------------------------------------

AC_ARG_WITH(includes,
   AC_HELP_STRING([--with-includes=string], [extra includes list for codels]),
   [
      if test "x$withval" != "xno" -a "x$withval" != "x"; then
         EXTRA_INCLUDES="$withval"
      fi
   ], [EXTRA_INCLUDES=])
AC_SUBST(EXTRA_INCLUDES)

AC_ARG_WITH(libraries,
   AC_HELP_STRING([--with-libraries=string], [extra libraries list for codels]),
   [
      if test "x$withval" != "xno" -a "x$withval" != "x"; then
         EXTRA_LIBS="$withval"
      fi
   ], [EXTRA_LIBS=])
AC_SUBST(EXTRA_LIBS)


PKG_CHECK_MODULES(GENBASIC, genBasic)
AC_SUBST(GENBASIC_CFLAGS)
AC_SUBST(GENBASIC_LIBS)


PKG_CHECK_MODULES(GENMANIP, genManip)
AC_SUBST(GENMANIP_CFLAGS)
AC_SUBST(GENMANIP_LIBS)


PKG_CHECK_MODULES(GENHUM, genHum)
AC_SUBST(GENHUM_CFLAGS)
AC_SUBST(GENHUM_LIBS)


PKG_CHECK_MODULES(GBM, gbM)
AC_SUBST(GBM_CFLAGS)
AC_SUBST(GBM_LIBS)


PKG_CHECK_MODULES(SOFTMOTION_LIBS, softMotion-libs >= 3.4)
AC_SUBST(SOFTMOTION_LIBS_CFLAGS)
AC_SUBST(SOFTMOTION_LIBS_LIBS)



dnl ============ Include the user part
m4_include([../configure.ac.user])
dnl ============ Include the user part (end)

AC_SUBST(EXTRA_SUBDIRS)
AC_SUBST(EXTRA_PKGCONFIG)

AC_CONFIG_FILES(
	[config.mk:autoconf/config.mk.in]
	[Makefile]
	[codels/Makefile]
	[server/Makefile]
	[pr2SoftMotion.pc:autoconf/pr2SoftMotion.pc.in]
	[local.mk]
)

for dir in $EXTRA_SUBDIRS; do
    AC_CONFIG_FILES([$dir/Makefile])
done
for pc in $EXTRA_PKGCONFIG; do
    AC_CONFIG_FILES([$pc.pc])
done

AC_OUTPUT

