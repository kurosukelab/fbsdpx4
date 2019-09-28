KMOD=	fbsdpx4
#SRCS=   opt_bus.h opt_usb.h device_if.h bus_if.h usb_if.h vnode_if.h usbdevs.h
SRCS=   opt_usb.h usb_if.h device_if.h bus_if.h usbdevs.h
SRCS+=  fbsdpx4.c it930x-bus.c it930x.c r850.c rt710.c tc90522.c
#SRCS+=	ringbuffer.c

CFLAGS+= -I../include 
CFLAGS+= -g -DDEBUG
CFLAGS.it930x.c+= -Wno-error=cast-qual
CFLAGS.r850.c+= -Wno-error=unused-const-variable

.include <bsd.kmod.mk>
