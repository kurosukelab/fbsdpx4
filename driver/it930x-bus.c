// it930x-bus.c

// IT930x bus functions
#if defined(__FreeBSD__)
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/mutex.h>
#include <sys/condvar.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbhid.h>
#include <dev/usb/usb_core.h>
#include "usbdevs.h"

#include "px4_misc.h"

#define IT930X_BUS_DISCARD_PACKETS (0);

#else
#include "print_format.h"

#include <linux/version.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/atomic.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/usb.h>
#endif

#include "it930x-config.h"
#include "it930x-bus.h"

struct it930x_usb_context;

struct it930x_usb_work {
	struct it930x_usb_context *ctx;
	struct urb *urb;
#ifdef IT930X_BUS_USE_WORKQUEUE
	struct work_struct work;
#endif
};

struct it930x_usb_context {
	struct it930x_bus *bus;
	it930x_bus_on_stream_t on_stream;
	void *ctx;
	u32 num_urb;
	bool no_dma;
#ifdef IT930X_BUS_USE_WORKQUEUE
	struct workqueue_struct *wq;
#endif
	struct it930x_usb_work *works;
	atomic_t start;
};

#if defined(__FreeBSD__)

void it930x_usb_ctrl_tx_msg_callback( struct usb_xfer *transfer, usb_error_t error );
void it930x_usb_ctrl_rx_msg_callback( struct usb_xfer *transfer, usb_error_t error );
void it930x_usb_stream_callback(struct usb_xfer *transfer, usb_error_t error);

static struct usb_config it930x_ctrl_config[ IT930X_BUS_CTRL_N_TRANSFER] = {
	[IT930X_BUS_CTRL_WR] = {
		.callback = &it930x_usb_ctrl_tx_msg_callback,
		.bufsize = IT930X_BUS_BUF_SIZE,
		.flags = { .pipe_bof = 1 },
		.type = UE_BULK,
		.endpoint = 0x02,
		.direction = UE_DIR_OUT
	},
	[IT930X_BUS_CTRL_RD] = {
		.callback = &it930x_usb_ctrl_rx_msg_callback,
		.bufsize = IT930X_BUS_BUF_SIZE,
		.flags = { .short_xfer_ok = 1, .pipe_bof = 1 },
		.type = UE_BULK,
		.endpoint = 0x81,
		.direction = UE_DIR_IN
	}
};
static struct usb_config it930x_stream_config[ IT930X_BUS_STREAM_N_TRANSFER] = {
	[IT930X_BUS_STREAM_RD] = {
		.callback = &it930x_usb_stream_callback,
		.bufsize = 188*816,
		.flags = { .short_xfer_ok = 1, .pipe_bof = 1, .proxy_buffer = 1 },
		.timeout = 1000, /* 1 second. */
		.type = UE_BULK,
		.endpoint = 0x84,
		.direction = UE_DIR_IN
	}
};

static struct it930x_bus_cmd_buf* it930x_bus_cmd_buf_alloc_locked( struct it930x_bus *bus, int flags )
{
	struct it930x_bus_cmd_buf *cb;

	while (( cb = TAILQ_FIRST( &bus->usb.cmd_buf_free )) == NULL ){
		if( flags != M_WAITOK ){
			break;
		}
		cv_wait( &bus->usb.xfer_cv, &bus->usb.xfer_mtx );
	}
	if( cb != NULL ){
		TAILQ_REMOVE( &bus->usb.cmd_buf_free, cb, entry );
		cb->len = 0;
	}
	return cb;
}

static struct it930x_bus_cmd_buf* it930x_bus_cmd_buf_alloc( struct it930x_bus *bus, int flags )
{
	struct it930x_bus_cmd_buf *cb;

	mtx_lock( &bus->usb.xfer_mtx );
	cb = it930x_bus_cmd_buf_alloc_locked( bus, flags );
	mtx_unlock( &bus->usb.xfer_mtx );

	return cb;
}

void it930x_usb_ctrl_tx_msg_callback( struct usb_xfer *transfer, usb_error_t error )
{
	struct it930x_bus *bus = usbd_xfer_softc( transfer );
	struct it930x_bus_cmd_head *phead = usbd_xfer_get_priv( transfer );
	struct it930x_bus_cmd_buf *cb;
	struct usb_page_cache *pc;
	
	switch (USB_GET_STATE(transfer)) {
	case USB_ST_TRANSFERRED:
		TAILQ_CONCAT( &bus->usb.cmd_buf_free, phead, entry );
		bus->usb.event |= IT930X_BUS_TRANSFERRED;
		//break;
	case USB_ST_SETUP:
	tr_setup:
		cb = TAILQ_FIRST( &bus->usb.cmd_tx_buf_pending );
		if(cb){
			TAILQ_REMOVE( &bus->usb.cmd_tx_buf_pending, cb, entry );
			TAILQ_INSERT_TAIL( phead, cb, entry );

			pc = usbd_xfer_get_frame( transfer, 0);
			usbd_copy_in( pc, 0, cb->buf, cb->len );
			usbd_xfer_set_frame_len( transfer, 0 , cb->len );
			
			usbd_xfer_set_frames( transfer, 1 );
			usbd_xfer_set_frame_data( transfer, 0, cb->buf, cb->len);
			usbd_transfer_submit(transfer);
		}
		break;
	default:
		TAILQ_CONCAT( &bus->usb.cmd_buf_free, phead, entry );
		if( error != USB_ERR_CANCELLED ) {
			usbd_xfer_set_stall( transfer );
			if(error){
				dev_err( bus->dev, "%s:tx error=%d\n", __FUNCTION__, error);
			}
			goto tr_setup;
		}
		bus->usb.event = -1;
		break;
	}
	cv_signal(&bus->usb.xfer_cv);
}

void it930x_usb_ctrl_rx_msg_callback( struct usb_xfer *transfer, usb_error_t error )
{
	struct it930x_bus *bus = usbd_xfer_softc( transfer );
	struct it930x_bus_cmd_head *phead = usbd_xfer_get_priv( transfer );
	struct it930x_bus_cmd_buf *cb;
	int actual;
	int max;
	
	usbd_xfer_status( transfer, &actual, NULL, NULL, NULL );
	
	switch (USB_GET_STATE(transfer)) {
	case USB_ST_TRANSFERRED:
		TAILQ_CONCAT( &bus->usb.cmd_buf_free, phead, entry );
		bus->usb.event |= IT930X_BUS_TRANSFERRED;
		//break;
	case USB_ST_SETUP:
	tr_setup:
		cb = TAILQ_FIRST( &bus->usb.cmd_rx_buf_pending );
		if(cb == NULL){
			cv_signal(&bus->usb.xfer_cv);
			return;
		}
		
		TAILQ_REMOVE( &bus->usb.cmd_rx_buf_pending, cb, entry );
		TAILQ_INSERT_TAIL( phead, cb, entry );

		max = usbd_xfer_max_len( transfer );
		usbd_xfer_set_frame_len( transfer, 0, max );
		usbd_transfer_submit(transfer);
		
		break;
	default:
		TAILQ_CONCAT( &bus->usb.cmd_buf_free, phead, entry );
		if( error != USB_ERR_CANCELLED ){
			usbd_xfer_set_stall( transfer );
			dev_err(bus->dev, "%s:rx error=%d\n", __FUNCTION__, error);
			goto tr_setup;
		}
		bus->usb.event = -1;
		break;
	}
	cv_signal(&bus->usb.xfer_cv);
}

void it930x_usb_stream_callback(struct usb_xfer *transfer, usb_error_t error)
{
	struct it930x_bus *bus = usbd_xfer_softc( transfer );
	struct it930x_usb_context *ctx = bus->usb.priv;
	struct usb_page_cache *pc;
	void *context = ctx->ctx;
	int actual;
	int max;
	
	usbd_xfer_status( transfer, &actual, NULL, NULL, NULL );
	
	switch (USB_GET_STATE(transfer)) {
	case USB_ST_TRANSFERRED:
		if( actual == 0 ){
			bus->usb.zero_length_packets++;
			//usbd_xfer_set_interval(transfer, 30);
		}
		else {
			if(bus->usb.zero_length_packets){
				dev_dbg(bus->dev, "zero length packets=%d\n", bus->usb.zero_length_packets);
			}
			usbd_xfer_set_interval(transfer, 0);
			bus->usb.zero_length_packets = 0;
			pc = usbd_xfer_get_frame( transfer, 0 );

			if(bus->usb.discard_packets > 0 ){
				bus->usb.discard_packets--;
			}
			
			if(!bus->usb.discard_packets){
				ctx->on_stream(context, pc , actual);
			}
		}
	case USB_ST_SETUP:
	tr_setup:
		if((bus->usb.fifos_put_bytes_max != NULL) && ( context != NULL )){
			if((*bus->usb.fifos_put_bytes_max)(context)){
				max = usbd_xfer_max_len( transfer );
				usbd_xfer_set_frame_len( transfer, 0, max );
				usbd_transfer_submit(transfer);
			}
			else {
				dev_dbg(bus->dev, "stream fifo has no space\n");
			}
		}
		else {
			dev_dbg(bus->dev, "stream fifos_put_bytes_max=%p context=%p\n", bus->usb.fifos_put_bytes_max, context);
		}
		
		break;
	default:
		usbd_xfer_set_interval(transfer, 0);
		bus->usb.zero_length_packets = 0;
		
		if( error != USB_ERR_CANCELLED ){
			usbd_xfer_set_stall( transfer );
			goto tr_setup;
		}
		break;
	}
}

static int it930x_usb_ctrl_tx_msg(struct it930x_bus *bus, const void *buf, int len, int *act_len, int timeout )
{
	struct usb_xfer *xfer;
	struct it930x_bus_cmd_buf *cb;
	int max;
	int ret;
	
	xfer= bus->usb.ctrl_transfer[ IT930X_BUS_CTRL_WR ];

	cb = it930x_bus_cmd_buf_alloc( bus, M_WAITOK );


	max = usbd_xfer_max_len( xfer);
	if ( len > max){
		len = max;
	}
	memcpy(cb->buf, buf, len);
	cb->len= len;
	
	*act_len= len;
		
	mtx_lock( &bus->usb.xfer_mtx );

	TAILQ_INSERT_TAIL( &bus->usb.cmd_tx_buf_pending, cb, entry );
	usbd_xfer_set_timeout( xfer, timeout );
	usbd_transfer_start( xfer );
	
	while( bus->usb.event == 0 ){
		cv_wait(&bus->usb.xfer_cv, &bus->usb.xfer_mtx);
	}
	bus->usb.event = 0;
	
	mtx_unlock( &bus->usb.xfer_mtx );

	ret= xfer->error;
	if(ret){
		dev_err( bus->dev, "%s:tx error=%d\n", __FUNCTION__, ret);
		usbd_transfer_stop( xfer );
	}


	return ret;
}

static int it930x_usb_ctrl_rx_msg(struct it930x_bus *bus, void *buf, int len, int *act_len, int timeout )
{
	struct usb_xfer *xfer;
	struct usb_page_cache *pc;
	struct it930x_bus_cmd_buf *cb;
	int ret;
	
	xfer= bus->usb.ctrl_transfer[ IT930X_BUS_CTRL_RD ];
	cb = it930x_bus_cmd_buf_alloc( bus, M_WAITOK );

	mtx_lock( &bus->usb.xfer_mtx );
	
	TAILQ_INSERT_TAIL( &bus->usb.cmd_rx_buf_pending, cb, entry );

	usbd_xfer_set_timeout(xfer, timeout );
	
	usbd_transfer_start( xfer );

	while( bus->usb.event == 0 ){
		cv_wait(&bus->usb.xfer_cv, &bus->usb.xfer_mtx );
	}
	bus->usb.event = 0;

	mtx_unlock( &bus->usb.xfer_mtx );
	
	ret= xfer->error;
	if( !ret ){
		pc = usbd_xfer_get_frame( xfer, 0 );
		*act_len = usbd_xfer_frame_len( xfer, 0 );
		if(*act_len <= len ){
			len= *act_len;
		}
		else {
			dev_dbg(bus->dev,"actual length=%d, len=%d\n", *act_len, len);
			*act_len= len;
		}
		usbd_copy_out( pc, 0, buf, len );
	}
	else {
		usbd_transfer_stop( xfer );
	}

	
	return ret;
}
#endif

static int it930x_usb_ctrl_tx(struct it930x_bus *bus, const void *buf, int len, void *opt)
{
	int ret = 0, rlen = 0;
#ifndef __FreeBSD__	
	struct usb_device *dev = bus->usb.dev;
#endif
#if 0
	const u8 *p = buf;
#endif

	if (len > IT930X_USB_MAX_CONTROL_TRANSFER_SIZE || !buf || !len)
		return -EINVAL;

#if 0
	while (len > 0) {
		int s = (len < IT930X_USB_MAX_CONTROL_PACKET_SIZE) ? len : IT930X_USB_MAX_CONTROL_PACKET_SIZE;

		ret = usb_bulk_msg(dev, usb_sndbulkpipe(dev, 0x02), p, s, &rlen, bus->usb.timeout);
		if (ret)
			break;

		p += rlen;
		len -= rlen;
	}
#else
	/* Endpoint 0x02: Control IN */
#if defined(__FreeBSD__)
	ret = it930x_usb_ctrl_tx_msg(bus, buf, len, &rlen, bus->usb.ctrl_timeout );
#else
	ret = usb_bulk_msg(dev, usb_sndbulkpipe(dev, 0x02), (void *)buf, len, &rlen, bus->usb.ctrl_timeout);
#endif
#endif

	if (ret)
		dev_dbg(bus->dev, "it930x_usb_ctrl_tx: Failed. (ret: %d)\n", ret);

	mdelay(1);
	
	return ret;
}

static int it930x_usb_ctrl_rx(struct it930x_bus *bus, void *buf, int *len, void *opt)
{
	int ret = 0, rlen = 0;
#ifndef __FreeBSD__
	struct usb_device *dev = bus->usb.dev;
#endif
	
	if (!buf || !len || !*len)
		return -EINVAL;

	/* Endpoint 0x81: Control OUT */
#if defined(__FreeBSD__)
	ret = it930x_usb_ctrl_rx_msg(bus, buf, *len, &rlen, bus->usb.ctrl_timeout );
#else
	ret = usb_bulk_msg(dev, usb_rcvbulkpipe(dev, 0x81), buf, *len, &rlen, bus->usb.ctrl_timeout);
#endif
	if (ret)
		dev_dbg(bus->dev, "it930x_usb_ctrl_rx: Failed. (ret: %d)\n", ret);

	*len = rlen;

	mdelay(1);
	
	return ret;
}

static int it930x_usb_stream_rx(struct it930x_bus *bus, void *buf, int *len, int timeout)
{
	int ret = 0;
#if !defined(__FreeBSD__)
	int rlen = 0;
	struct usb_device *dev = bus->usb.dev;

	if (!buf | !len || !*len)
		return -EINVAL;

	/* Endpoint 0x84: Stream OUT */
	ret = usb_bulk_msg(dev, usb_rcvbulkpipe(dev, 0x84), buf, *len, &rlen, timeout);
	if (ret)
		dev_dbg(bus->dev, "it930x_usb_stream_rx: Failed. (ret: %d)\n", ret);

	*len = rlen;
#endif

	return ret;
}

#if !defined(__FreeBSD__)
static void free_urb_buffers(struct usb_device *dev, struct it930x_usb_work *works, u32 n, bool free_urb, bool no_dma)
{

	u32 i;

	if (!works)
		return;

	for (i = 0; i < n; i++) {
		struct urb *urb = works[i].urb;

		if (!urb)
			continue;

		if (urb->transfer_buffer) {
			if (!no_dma)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
				usb_free_coherent(dev, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma);
#else
				usb_buffer_free(dev, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma);
#endif
			else
				kfree(urb->transfer_buffer);

			urb->transfer_buffer = NULL;
			urb->transfer_buffer_length = 0;
		}

		if (free_urb) {
			usb_free_urb(urb);
			works[i].urb = NULL;
		}
	}

	return;
}
#endif

#ifdef IT930X_BUS_USE_WORKQUEUE
static void it930x_usb_workqueue_handler(struct work_struct *work)
{
	struct it930x_usb_work *w = container_of(work, struct it930x_usb_work, work);
	struct it930x_usb_context *ctx = w->ctx;
	int ret = 0;

	ret = usb_submit_urb(w->urb, GFP_ATOMIC);
	if (ret)
		dev_dbg(ctx->bus->dev, "it930x_usb_workqueue_handler: usb_submit_urb() failed. (ret: %d)\n", ret);
}
#endif

#if !defined(__FreeBSD__)
static void it930x_usb_complete(struct urb *urb)
{

	int ret = 0;
	struct it930x_usb_work *w = urb->context;
	struct it930x_usb_context *ctx = w->ctx;

	if (urb->status) {
		dev_dbg(ctx->bus->dev, "it930x_usb_complete: status: %d\n", urb->status);
		return;
	}

	if (urb->actual_length)
		ret = ctx->on_stream(ctx->ctx, urb->transfer_buffer, urb->actual_length);
	else
		dev_dbg(ctx->bus->dev, "it930x_usb_complete: !urb->actual_length\n");

	if (!ret && (atomic_read(&ctx->start) == 1)) {
#ifdef IT930X_BUS_USE_WORKQUEUE
		ret = queue_work(ctx->wq, &w->work);
		if (ret)
			dev_dbg(ctx->bus->dev, "it930x_usb_complete: queue_work() failed. (ret: %d)\n", ret);
#else
		ret = usb_submit_urb(urb, GFP_ATOMIC);
		if (ret)
			dev_dbg(ctx->bus->dev, "it930x_usb_complete: usb_submit_urb() failed. (ret: %d)\n", ret);
#endif
	}
	
	return;
}
#endif

static int it930x_usb_start_streaming(struct it930x_bus *bus, it930x_bus_on_stream_t on_stream, void *context)
{
	int ret = 0;

#if !defined(__FreeBSD__)
	u32 i, l, n;
	bool no_dma;
	struct usb_device *dev = bus->usb.dev;
#endif
	struct it930x_usb_context *ctx = bus->usb.priv;
#if !defined(__FreeBSD__)
	struct it930x_usb_work *works;
#endif

	if (!on_stream)
		return -EINVAL;

	dev_dbg(bus->dev, "it930x_usb_start_streaming\n");

	if (atomic_add_return(2, &ctx->start) != 2) {
		atomic_sub(2, &ctx->start);
		return 0;
	}

#if !defined(__FreeBSD__)
	l = bus->usb.streaming_urb_buffer_size;
	n = bus->usb.streaming_urb_num;
	no_dma = bus->usb.streaming_no_dma;
#endif
	
	ctx->on_stream = on_stream;
	ctx->ctx = context;

#if !defined(__FreeBSD__)
	works = kcalloc(n, sizeof(*works), GFP_ATOMIC);
	if (!works) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < n; i++) {
		struct urb *urb;
		void *p;
		dma_addr_t dma;

		urb = usb_alloc_urb(0, GFP_ATOMIC | __GFP_ZERO);
		if (!urb) {
			dev_err(bus->dev, "it930x_usb_start_streaming: usb_alloc_urb() failed. (i: %u)\n", i);
			break;
		}

		if (!no_dma)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
			p = usb_alloc_coherent(dev, l, GFP_ATOMIC, &dma);
#else
			p = usb_buffer_alloc(dev, l, GFP_ATOMIC, &dma);
#endif
		else
			p = kmalloc(l, GFP_ATOMIC);

		if (!p) {
			if (!no_dma)
				dev_err(bus->dev, "it930x_usb_start_streaming: usb_alloc_coherent() failed. (i: %u)\n", i);
			else
				dev_err(bus->dev, "it930x_usb_start_streaming: kmalloc() failed. (i: %u)\n", i);

			usb_free_urb(urb);
			break;
		}

		dev_dbg(bus->dev, "it930x_usb_start_streaming: p: %p, l: %u, dma: %pad\n", p, l, &dma);

		usb_fill_bulk_urb(urb, dev, usb_rcvbulkpipe(dev, 0x84), p, l, it930x_usb_complete, &works[i]);

		if (!no_dma) {
			urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
			urb->transfer_dma = dma;
		}

		works[i].ctx = ctx;
		works[i].urb = urb;
#ifdef IT930X_BUS_USE_WORKQUEUE
		INIT_WORK(&works[i].work, it930x_usb_workqueue_handler);
#endif
	}

	n = i;

	if (!n) {
		ret = -ENOMEM;
		goto fail;
	}

#ifdef IT930X_BUS_USE_WORKQUEUE
	ctx->wq = create_singlethread_workqueue("it930x_usb_workqueue");
	if (!ctx->wq)
		goto fail;
#endif

#endif

#if defined(__FreeBSD__)
	bus->usb.discard_packets = IT930X_BUS_DISCARD_PACKETS;
	usbd_xfer_set_stall(bus->usb.stream_transfer[ IT930X_BUS_STREAM_RD ]);
	usbd_transfer_start( bus->usb.stream_transfer[ IT930X_BUS_STREAM_RD ] );
#else
	usb_reset_endpoint(dev, 0x84);
	
	for (i = 0; i < n; i++) {
		ret = usb_submit_urb(works[i].urb, GFP_ATOMIC);
		if (ret) {
			int j;

			dev_err(bus->dev, "it930x_usb_start_streaming: usb_submit_urb() failed. (i: %u, ret: %d)\n", i, ret);

			for (j = 0; j < i; j++)
				usb_kill_urb(works[i].urb);

			break;
		}
	}

	if (ret)
		goto fail;
#endif

#if !defined(__FreeBSD__)
	dev_dbg(bus->dev, "it930x_usb_start_streaming: n: %u\n", n);

	ctx->num_urb = n;
	ctx->no_dma = no_dma;
	ctx->works = works;
#endif
	
	atomic_sub(1, &ctx->start);

	return ret;

#if !defined(__FreeBSD__)	
fail:
#ifdef IT930X_BUS_USE_WORKQUEUE
	if (ctx->wq) {
		flush_workqueue(ctx->wq);
		destroy_workqueue(ctx->wq);
	}
#endif

	free_urb_buffers(dev, works, n, true, no_dma);

	if (works)
		kfree(works);
	
	ctx->on_stream = NULL;
	ctx->ctx = NULL;
	ctx->num_urb = 0;
	ctx->no_dma = false;
#ifdef IT930X_BUS_USE_WORKQUEUE
	ctx->wq = NULL;
#endif
	ctx->works = NULL;

	atomic_sub(2, &ctx->start);

	return ret;
#endif
}

static int it930x_usb_stop_streaming(struct it930x_bus *bus)
{
#if !defined(__FreeBSD__)
	u32 i, n;
	struct usb_device *dev = bus->usb.dev;
#endif	
	struct it930x_usb_context *ctx = bus->usb.priv;
#if !defined(__FreeBSD__)
	struct it930x_usb_work *works = ctx->works;
#endif
	
	dev_dbg(bus->dev, "it930x_usb_stop_streaming\n");

	if (atomic_sub_return(2, &ctx->start) != -1) {
		atomic_add(2, &ctx->start);
		return 0;
	}

#if defined(__FreeBSD__)
	usbd_transfer_stop( bus->usb.stream_transfer[ IT930X_BUS_STREAM_RD ] );
#else
	n = ctx->num_urb;

#ifdef IT930X_BUS_USE_WORKQUEUE
	if (ctx->wq) {
		flush_workqueue(ctx->wq);
		destroy_workqueue(ctx->wq);
	}
#endif

	if (works) {
		for (i = 0; i < n; i++)
			usb_kill_urb(works[i].urb);

		free_urb_buffers(dev, works, n, true, ctx->no_dma);
		kfree(works);
	}
#endif
	
	ctx->on_stream = NULL;
	ctx->ctx = NULL;
#if !defined(__FreeBSD__)
	ctx->num_urb = 0;
	ctx->no_dma = false;
#ifdef IT930X_BUS_USE_WORKQUEUE
	ctx->wq = NULL;
#endif
	ctx->works = NULL;
#endif
	
	atomic_add(1, &ctx->start);

	return 0;
}

int it930x_bus_init(struct it930x_bus *bus)
{
	int ret = 0;

	if (!bus)
		return -EINVAL;

	switch(bus->type) {
	case IT930X_BUS_USB:
		if (!bus->usb.dev) {
			ret = -EINVAL;
		} else {
			struct it930x_usb_context *ctx;

			ctx = kmalloc(sizeof(*ctx), GFP_ATOMIC);
			if (!ctx) {
				ret = -ENOMEM;
				break;
			}

#if !defined(__FreeBSD__)
			usb_get_dev(bus->usb.dev);
#endif
			
			ctx->bus = bus;
			ctx->on_stream = NULL;
			ctx->ctx = NULL;
			ctx->num_urb = 0;
			ctx->no_dma = false;
#ifdef IT930X_BUS_USE_WORKQUEUE
			ctx->wq = NULL;
#endif
			ctx->works = NULL;
			atomic_set(&ctx->start, 0);

			bus->usb.priv = ctx;

			bus->ops.ctrl_tx = it930x_usb_ctrl_tx;
			bus->ops.ctrl_rx = it930x_usb_ctrl_rx;
			bus->ops.stream_rx = it930x_usb_stream_rx;
			bus->ops.start_streaming = it930x_usb_start_streaming;
			bus->ops.stop_streaming = it930x_usb_stop_streaming;

#if defined(__FreeBSD__)
			{
				int error;
				int i;
				
				cv_init( &bus->usb.xfer_cv, "it930x-bus");
				mtx_init( &bus->usb.xfer_mtx, "it930x-bus", NULL, MTX_DEF | MTX_RECURSE);
				mtx_init( &bus->usb.stream_mtx, "it930x-bus(stream)", NULL, MTX_DEF | MTX_RECURSE);
				bus->usb.event = 0;
				bus->usb.discard_packets = IT930X_BUS_DISCARD_PACKETS;
				
				it930x_stream_config[IT930X_BUS_STREAM_RD].bufsize = bus->usb.streaming_usb_buffer_size;
				
				error = usbd_transfer_setup( bus->usb.dev, &bus->usb.iface_index,
											 bus->usb.ctrl_transfer, it930x_ctrl_config, IT930X_BUS_CTRL_N_TRANSFER, bus,
											 &bus->usb.xfer_mtx );
				
				if (error) {
					ret = -ENOMEM;
					break;
				}

				error = usbd_transfer_setup( bus->usb.dev, &bus->usb.iface_index,
											 bus->usb.stream_transfer, it930x_stream_config, IT930X_BUS_STREAM_N_TRANSFER, bus,
											 &bus->usb.stream_mtx );
				
				if (error) {
					ret = -ENOMEM;
					break;
				}
				dev_dbg(bus->dev, "%s:stream bufsize=%d\n", __FUNCTION__, it930x_stream_config[IT930X_BUS_STREAM_RD].bufsize);

				usbd_xfer_set_priv( bus->usb.ctrl_transfer[ IT930X_BUS_CTRL_WR ], &bus->usb.xfer_tx_head );
				usbd_xfer_set_priv( bus->usb.ctrl_transfer[ IT930X_BUS_CTRL_RD ], &bus->usb.xfer_rx_head );
				TAILQ_INIT(&bus->usb.xfer_tx_head);
				TAILQ_INIT(&bus->usb.cmd_buf_free);
				TAILQ_INIT(&bus->usb.cmd_tx_buf_pending);
				TAILQ_INIT(&bus->usb.xfer_rx_head);
				TAILQ_INIT(&bus->usb.cmd_rx_buf_pending);

				for( i = 0; i < IT930X_BUS_CMD_MAX_BUFFERS; i++ ){
					struct it930x_bus_cmd_buf *cb = &bus->usb.cmd_buf[i];
					TAILQ_INSERT_TAIL( &bus->usb.cmd_buf_free, cb, entry );
				}
			}
#endif

		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int it930x_bus_term(struct it930x_bus *bus)
{
	int ret = 0;

	if (!bus) {
		ret = -EINVAL;
		goto exit;
	}

	switch(bus->type) {
	case IT930X_BUS_USB:
	{
		struct it930x_usb_context *ctx = bus->usb.priv;

		if (ctx) {
			it930x_usb_stop_streaming(bus);
			kfree(ctx);
		}
#if defined(__FreeBSD__)
		if (bus->usb.dev){
			usbd_transfer_unsetup( bus->usb.stream_transfer, IT930X_BUS_STREAM_N_TRANSFER );
			usbd_transfer_unsetup( bus->usb.ctrl_transfer, IT930X_BUS_CTRL_N_TRANSFER );
			mtx_destroy( &bus->usb.stream_mtx );
			cv_destroy( &bus->usb.xfer_cv );
			mtx_destroy( &bus->usb.xfer_mtx );
		}
#else  
		if (bus->usb.dev)
			usb_put_dev(bus->usb.dev);
#endif
		
		break;
	}

	default:
		break;
	}

	memset(bus, 0, sizeof(struct it930x_bus));

exit:
	return ret;
}
