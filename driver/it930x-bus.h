// it930x-bus.h

#ifndef	__IT930X_BUS_H__
#define __IT930X_BUS_H__

#if defined(__FreeBSD__)
#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

enum {
	IT930X_BUS_CTRL_WR,
	IT930X_BUS_CTRL_RD,
	IT930X_BUS_CTRL_N_TRANSFER
};

enum {
	IT930X_BUS_STREAM_RD,
	IT930X_BUS_STREAM_N_TRANSFER
};

#define IT930X_BUS_TRANSFERRED (1)
#define IT930X_BUS_BUF_SIZE (256)
#define IT930X_BUS_CMD_MAX_BUFFERS (16)

#else
#include <linux/device.h>
#include <linux/usb.h>
#endif

#include "it930x-config.h"

typedef enum {
	IT930X_BUS_NONE = 0,
	IT930X_BUS_USB,
} it930x_bus_type_t;

#if defined(__FreeBSD__)
typedef int (*it930x_bus_on_stream_t)(void *context, struct usb_page_cache *pc, u32 len);
#else
typedef int (*it930x_bus_on_stream_t)(void *context, void *buf, u32 len);
#endif

struct it930x_bus;

struct it930x_bus_operations {
	int (*ctrl_tx)(struct it930x_bus *bus, const void *buf, int len, void *opt);
	int (*ctrl_rx)(struct it930x_bus *bus, void *buf, int *len, void *opt);
	int (*stream_rx)(struct it930x_bus *bus, void *buf, int *len, int timeout);
	int (*start_streaming)(struct it930x_bus *bus, it930x_bus_on_stream_t on_stream, void *context);
	int (*stop_streaming)(struct it930x_bus *bus);
};
#if defined(__FreeBSD__)

struct it930x_bus_cmd_buf {
	TAILQ_ENTRY(it930x_bus_cmd_buf) entry;
	uint32_t len;
	uint8_t buf[ IT930X_BUS_BUF_SIZE ];
};

TAILQ_HEAD(it930x_bus_cmd_head, it930x_bus_cmd_buf);
  
#endif

struct it930x_bus {
#if defined(__FreeBSD__)
	device_t dev;
#else
	struct device *dev;
#endif
	it930x_bus_type_t type;
	union {
		struct {
			struct usb_device *dev;
			int ctrl_timeout;
#if !defined(__FreeBSD__)
			u32 streaming_urb_buffer_size;
			u32 streaming_urb_num;
			bool streaming_no_dma;
#endif
			void *priv;
#if defined(__FreeBSD__)
			u32 streaming_usb_mbufs;
			u32 streaming_usb_buffer_size;
			uint8_t iface_num;
			uint8_t iface_index;
			struct usb_xfer *ctrl_transfer[IT930X_BUS_CTRL_N_TRANSFER];
			struct usb_xfer *stream_transfer[IT930X_BUS_STREAM_N_TRANSFER];
			uint8_t zero_length_packets;
			uint8_t discard_packets;
			struct it930x_bus_cmd_head xfer_tx_head;
			struct it930x_bus_cmd_head cmd_buf_free;
			struct it930x_bus_cmd_head cmd_tx_buf_pending;
			struct it930x_bus_cmd_buf cmd_buf[ IT930X_BUS_CMD_MAX_BUFFERS ];
			struct it930x_bus_cmd_head xfer_rx_head;
			struct it930x_bus_cmd_head cmd_rx_buf_pending;
			struct cv xfer_cv;
			struct mtx xfer_mtx;
			struct mtx stream_mtx;
			u32 event;
#endif
		} usb;
	};
	struct it930x_bus_operations ops;
};

int it930x_bus_init(struct it930x_bus *bus);
int it930x_bus_term(struct it930x_bus *bus);
  
static inline int it930x_bus_ctrl_tx(struct it930x_bus *bus, const void *buf, int len, void *opt)
{
	if (!bus || !bus->ops.ctrl_tx)
		return -EINVAL;

	return bus->ops.ctrl_tx(bus, buf, len, opt);
}

static inline int it930x_bus_ctrl_rx(struct it930x_bus *bus, void *buf, int *len, void *opt)
{
	if (!bus || !bus->ops.ctrl_rx)
		return -EINVAL;

	return bus->ops.ctrl_rx(bus, buf, len, opt);
}

static inline int it930x_bus_stream_rx(struct it930x_bus *bus, void *buf, int *len, int timeout)
{
	if (!bus || !bus->ops.stream_rx)
		return -EINVAL;

	return bus->ops.stream_rx(bus, buf, len, timeout);
}

static inline int it930x_bus_start_streaming(struct it930x_bus *bus, it930x_bus_on_stream_t on_stream, void *context)
{
	if (!bus || !bus->ops.start_streaming)
		return -EINVAL;

	return bus->ops.start_streaming(bus, on_stream, context);
}

static inline int it930x_bus_stop_streaming(struct it930x_bus *bus)
{
	if (!bus || !bus->ops.stop_streaming)
		return -EINVAL;

	return bus->ops.stop_streaming(bus);
}

#endif
