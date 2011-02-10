
###### Default target
codels-mk-all: $(OBJDIR) $(OBJDIR)/$(MODULE_BIN)

vpath %.c  	$(srcdir)
vpath %.cc  	$(srcdir)

###### GenoM flags
include $(top_srcdir)/$(autoconfdir)/genom.mk

CPPFLAGS += -I. -I$(top_builddir) -I$(srcdir) -I$(top_srcdir)
CPPFLAGS += $(ENDIANNESS) $(GENOM_DEFINES) $(GENOM_INCLUDES)
CPPFLAGS += $(GENOM_CFLAGS) $(EXTRA_INCLUDES)

ifeq ($(LANG_CXX),yes)
codels_obj=\
	$(patsubst %.c,$(OBJDIR)/%.lo,\
	$(patsubst %.cpp,$(OBJDIR)/%.lo,\
	$(patsubst %.cc,$(OBJDIR)/%.lo,$(codels_src))))
else
codels_obj= $(codels_src:%.c=$(OBJDIR)/%.lo)
endif


###### Build

$(OBJDIR)/$(MODULE_BIN): $(OBJDIR)/$(USER_LIB)
$(OBJDIR)/$(MODULE_BIN): $(top_builddir)/$(serverdir)/$(OBJDIR)/$(MODULE_LIB)
$(OBJDIR)/$(MODULE_BIN): $(top_builddir)/$(serverdir)/$(OBJDIR)/$(POSTERWRITE_LIB)

$(OBJDIR)/$(MODULE_BIN):
	$(LTLD) $(BINEXPORT) $(CFLAGS) -o $@ $(LDFLAGS) $(LIBTOOL_LINKOPT)		\
		$(top_builddir)/$(serverdir)/$(OBJDIR)/$(MODULE_LIB)	\
		$(OBJDIR)/$(USER_LIB) \
		$(top_builddir)/$(serverdir)/$(OBJDIR)/$(CLIENT_LIB)	\
		$(CODEL_LIB) $(EXTRA_LIBS) \
		$(top_builddir)/$(serverdir)/$(OBJDIR)/$(POSTERWRITE_LIB)	\
		$(GENOM_LIBS)

$(OBJDIR)/$(USER_LIB): $(codels_obj)
	$(LTLD) $(CFLAGS) $(codels_obj) -o $@ $(LDFLAGS)

ifeq ($(LANG_CXX),yes)
$(OBJDIR)/%.lo: %.cpp
	$(LTCXX) -c $(CPPFLAGS) $(CODELS_CPPFLAGS) $(CFLAGS) -o $@ $< $(LIBTOOL_COPT)
$(OBJDIR)/%.lo: %.cc
	$(LTCXX) -c $(CPPFLAGS) $(CODELS_CPPFLAGS) $(CFLAGS) -o $@ $< $(LIBTOOL_COPT)
endif
$(OBJDIR)/%.lo: %.c
	$(LTCC) -c $(CPPFLAGS) $(CODELS_CPPFLAGS) $(CFLAGS) -o $@ $< $(LIBTOOL_COPT)

$(OBJDIR):
	mkdir -p $@

# --- install -----------------------------------------------------------

.PHONY: install install-bin

install: all install-bin user-install

user-install:
install-bin:
	$(LTINSTALL) $(OBJDIR)/$(MODULE_BIN) ${DESTDIR}$(bindir)/$(MODULE_BIN)


# --- clean -------------------------------------------------------------

.PHONY: clean

clean: 
	$(top_builddir)/libtool --mode=clean rm -f \
		$(codels_obj) \
		$(OBJDIR)/$(MODULE_BIN) \
		$(OBJDIR)/$(USER_LIB)
	rm -f $(OBJDIR)/dependencies 
	-if test -d $(OBJDIR); then rmdir $(OBJDIR); fi

distclean: clean
	rm -f Makefile

# --- dependencies ------------------------------------------------------

ifeq ($(findstring clean,$(MAKECMDGOALS)),)
.PHONY: depend
depend $(OBJDIR)/dependencies:: $(OBJDIR);

depend $(OBJDIR)/dependencies:: $(codels_src)
	$(MKDEP) -c$(CC) -o$(OBJDIR)/dependencies -d$(OBJDIR) -t.lo \
		$(CPPFLAGS) $?

-include $(OBJDIR)/dependencies
endif

