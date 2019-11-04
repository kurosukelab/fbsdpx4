KMOD=	px4
SRCS=   opt_usb.h usb_if.h device_if.h bus_if.h usbdevs.h
SRCS+=  fbsdpx4.c it930x-bus.c it930x.c r850.c rt710.c tc90522.c
SRCS+=	revision.h
#SRCS+=	ringbuffer.c

revision.h!
	rev=`git rev-list --count HEAD` 2>/dev/null; \
	rev_name=`git name-rev --name-only HEAD` 2>/dev/null; \
	commit=`git rev-list --max-count=1 HEAD` 2>/dev/null; \
	if [ ! -s $@ ] || [ \"`grep -soE '^#define REVISION_NUMBER[[:blank:]]+"[0-9]+"$$' $@ | sed -E 's/^.+"(.*)"$$/\1/g'`.`grep -soE '^#define REVISION_NAME[[:blank:]]+"[[:print:]]+"$$' $@ | sed -E 's/^.+"(.*)"$$/\1/g'`.`grep -soE '^#define COMMIT_HASH[[:blank:]]+"[0-9A-Fa-f]+"$$' $@ | sed -E 's/^.+"(.*)"$$/\1/g'`\" != \"$${rev}.$${rev_name}.$${commit}\" ]; then \
	echo "// revision.h" > $@; \
	echo "" >> $@; \
	echo "#ifndef __REVISION_H__" >> $@; \
	echo "#define __REVISION_H__" >> $@; \
	echo "" >> $@; \
	if [ -n "$${rev}" ]; then \
	echo "#define REVISION_NUMBER	\"$${rev}\"" >> $@; \
	fi; \
	if [ -n "$${rev_name}" ]; then \
	echo "#define REVISION_NAME	\"$${rev_name}\"" >> $@; \
	fi; \
	if [ -n "$${commit}" ]; then \
	echo "#define COMMIT_HASH	\"$${commit}\"" >> $@; \
	fi; \
	echo "" >> $@; \
	echo "#endif" >> $@; \
	echo "'revision.h' was updated."; \
	fi

CFLAGS+= -I../include 
CFLAGS+= -g -DDEBUG
CFLAGS.it930x.c+= -Wno-error=cast-qual
CFLAGS.r850.c+= -Wno-error=unused-const-variable

.include <bsd.kmod.mk>
