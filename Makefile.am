bin_PROGRAMS = cy3240_i2c runTests
lib_LTLIBRARIES = libcy3240.la

ACLOCAL_AMFLAGS= -I m4

AM_CPPFLAGS=-I aceunit/src/native
AM_CPPFLAGS+=-I src
AM_CPPFLAGS+=-DACEUNIT_SUITES
AM_CPPFLAGS+=-I/usr/lib/jvm/default-java/include
AM_CPPFLAGS+=-I/usr/lib/jvm/default-java/include/linux

# CY3240 Shared Library
libcy3240_la_SOURCES = \
	src/cy3240.c \
	src/cy3240.h \
	src/cy3240_util.c \
	src/cy3240_util.h \
	src/cy3240_debug.c \
	src/cy3240_debug.h \
	src/cy3240_packet.h \
	src/cy3240_private_types.h \
	src/cy3240_types.h \
	src/jni/native_cy3240bridgecontroller.c \
	src/jni/native_cy3240bridgecontroller.h

libcy3240_la_LIBADD= -lhidapi-libusb

# cy3240-i2c Test Application
cy3240_i2c_SOURCES = \
	src/main.c \
	src/i2c_demo.h

cy3240_i2c_LDADD= -lhidapi-libusb -lcy3240

# Unit Test Application
runTests_SOURCES = \
	src/cy3240_private_types.h \
	src/tests/Suite1.c \
	src/tests/writeTest.c \
	src/tests/writeTest.h \
	src/tests/readTest.c \
	src/tests/readTest.h \
	src/tests/reconfigTest.c \
	src/tests/reconfigTest.h \
	src/tests/unittest.h \
	src/tests/unittest.c \
	aceunit/src/native/AceUnit.c \
	aceunit/src/native/AceUnit.h \
	aceunit/src/native/AceUnitAssert.h \
	aceunit/src/native/AceUnitData.c \
	aceunit/src/native/AceUnitData.h \
	aceunit/src/native/FullPlainLogger.c \
	aceunit/src/native/AceUnitAnnotations.h \
	aceunit/src/native/AceUnitMain.c \
	aceunit/src/native/AceUnitLogging.h \
	aceunit/src/native/ExceptionHandling.c \
	aceunit/src/native/ExceptionHandling.h

runTests_LDADD= -lhidapi-libusb -lcy3240
