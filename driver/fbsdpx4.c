#include <sys/stdint.h>
#include <sys/stddef.h>
#include <sys/param.h>
#include <sys/queue.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/kthread.h>
#include <sys/proc.h>
#include <sys/sched.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/sysctl.h>
#include <sys/sx.h>
#include <sys/unistd.h>
#include <sys/callout.h>
#include <sys/malloc.h>
#include <sys/priv.h>
#include <sys/syslog.h>
#include <sys/selinfo.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/limits.h>
#include <machine/atomic.h>

#include <dev/usb/usb.h>
#include <dev/usb/usb_dev.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbhid.h>
#include <dev/usb/usb_core.h>
#include <dev/usb/usb_busdma.h>
#include "usbdevs.h"

#include "px4_misc.h"

#include "firmware.h"
#include "fbsdpx4.h"
#include "revision.h"
#include "ptx_ioctl.h"
#include "i2c_comm.h"
#include "it930x-bus.h"
#include "it930x.h"
#include "tc90522.h"
#include "r850.h"
#include "rt710.h"

#if !defined(MAX_DEVICE) || !MAX_DEVICE
#undef MAX_DEVICE
#define MAX_DEVICE	16
#endif
#define TSDEV_NUM	4
#define MAX_TSDEV	(MAX_DEVICE * TSDEV_NUM)
#define DEVICE_NAME	"px4"

#define PID_PX_W3U4	0x083f
#define PID_PX_W3PE4	0x023f
#define PID_PX_Q3U4	0x084a
#define PID_PX_Q3PE4	0x024a

#define ISDB_T	1
#define ISDB_S	2

#define TS_SYNC_COUNT	4
#define TS_SYNC_SIZE	(188 * TS_SYNC_COUNT)

#define PX4_MAX_DEV 4

//#define CONTINUITY_COUNTER_CHECK
#define STREAM_BUF_SIZE ( 188 * 816 )
#define STREAM_MAX_BUFFERS (6)

struct px4_tsdev {
	struct sx xlock;
	unsigned int id;
	int isdb;				// ISDB_T or ISDB_S
	bool init;
	bool open;
	bool lnb_power;
	struct tc90522_demod tc90522;
	union {
		struct r850_tuner r850;		// for ISDB-T
		struct rt710_tuner rt710;	// for ISDB-S
	} t;
	atomic_t streaming;			// 0: not streaming, !0: streaming
	atomic_t ts_packet_read;
	struct usb_fifo *sc_fifo_open;
	int freq;
	struct mtx fifo_lock;
	unsigned char *fifo_buffers;
	unsigned int  fifo_index;
	unsigned int  fifo_max_index;
	unsigned int  fifo_buffer_size;
	unsigned int  pending_fifo_size;
	
	unsigned int  discard_ts_packets;
	unsigned long int ts_packet_count;  // 8byte
#ifdef CONTINUITY_COUNTER_CHECK
	char continuity_counter[0x2000];
#endif
};

struct stream_buf {
	TAILQ_ENTRY( stream_buf ) entry;
	uint32_t len;
	uint8_t  buf[ STREAM_BUF_SIZE ];
};

TAILQ_HEAD( stream_head, stream_buf );
	
struct px4_stream_context {
	struct px4_tsdev *ptsdev[ TSDEV_NUM ];
	u8 remain_buf[ 2*TS_SYNC_SIZE ];
	size_t remain_len;
	u8 sync_flag;
	u8 resync_count;
	struct mtx mtx;
	struct cv cv;
	int event;
	struct stream_head stream_buf_list;
	struct stream_head stream_buf_free;
	struct stream_buf stream_buf[ STREAM_MAX_BUFFERS ];
	
};

struct px4_multi_device;

struct px4_softc {
	device_t dev;	
	struct usb_device *usbdev;
	//struct mtx lock;
	struct sx xlock;
	struct usb_callout sc_watchdog;
	uint8_t sc_iface_num;
	uint8_t sc_previous_status;
	int dev_idx;
	atomic_t ref;				// reference counter
	atomic_t avail;				// availability flag
	struct cv wait_cv;
	struct mtx wait_mtx;
	unsigned long long serial_number;
	unsigned int dev_id;			// 1 or 2
	struct px4_multi_device *multi_dev;
	struct it930x_bridge it930x;
	unsigned int lnb_power_count;
	unsigned int streaming_count;
	struct px4_tsdev tsdev[TSDEV_NUM];
	struct usb_fifo_sc sc_fifo[ TSDEV_NUM ];
	struct proc *kthread;
	struct px4_stream_context *stream_context;
	struct cdev cdev;
};

// for PX-Q3U4 and PX-Q3PE4
struct px4_multi_device {
	struct mtx lock;
	int ref;			// reference counter of this structure
	int power_count;
	struct px4_softc *devs[2];
};


static device_probe_t px4_probe;
static device_attach_t px4_attach;
static device_detach_t px4_detach;
static usb_fifo_open_t px4_tsdev_open;
static usb_fifo_close_t px4_tsdev_release;
//static usb_fifo_ioctl_t px4_tsdev_ioctl;
static usb_fifo_cmd_t px4_tsdev_start_read;
static usb_fifo_cmd_t px4_tsdev_stop_read;

static struct sx glock;
SX_SYSINIT(px4_lock, &glock, "px4 global lock");
//static struct class *px4_class = NULL;
//static dev_t px4_dev_first;
static struct px4_softc *devs[ MAX_DEVICE ];
static unsigned int xfer_packets = 816;
static unsigned int usb_max_frames = 1;
static unsigned int usb_max_mbufs = 6;
static unsigned int tsdev_max_packets = 2048;
static bool disable_multi_device_power_control = true;
bool s_tuner_no_sleep = false;
unsigned int px4_debug = 4;
//static unsigned int discard_ts_packets = 2048;
static unsigned int discard_ts_packets = 0;

static struct usb_fifo_methods px4_fifo_methods = {
	.f_open = &px4_tsdev_open,
	.f_close = &px4_tsdev_release,
	//.f_ioctl = &px4_tsdev_ioctl,
	.f_start_read = &px4_tsdev_start_read,
	.f_stop_read = &px4_tsdev_stop_read,
	.basename[0] = "px4video",
	.postfix[0] = "s"
};

static void stream_thread( void *arg );
static int px4_sysctl_debug(SYSCTL_HANDLER_ARGS);
static int px4_sysctl_tsdev_max_packets(SYSCTL_HANDLER_ARGS);
static int px4_sysctl_xfer_packets(SYSCTL_HANDLER_ARGS);
static int px4_sysctl_usb_max_frames(SYSCTL_HANDLER_ARGS);
static int px4_sysctl_usb_max_mbufs(SYSCTL_HANDLER_ARGS);
static int px4_sysctl_discard_ts_packets(SYSCTL_HANDLER_ARGS);
static int px4_sysctl_lnb(SYSCTL_HANDLER_ARGS);
static int px4_sysctl_freq(SYSCTL_HANDLER_ARGS);
static int px4_sysctl_signal(SYSCTL_HANDLER_ARGS);

static void px4_sysctl_init(void *p)
{
	struct px4_softc *px4;
	device_t dev;
	struct sysctl_ctx_list *scl;
	struct sysctl_oid_list *sol;
	struct sysctl_oid *soid;

	px4 = p;
	dev = px4->dev;

	scl = device_get_sysctl_ctx( dev );
	sol = SYSCTL_CHILDREN( device_get_sysctl_tree( dev ) );
	
	//SYSCTL_ADD_PROC(scl, sol,
	//				OID_AUTO, "lnb", CTLTYPE_INT|CTLFLAG_RW|CTLFLAG_ANYBODY,
	//				px4, 0, px4_sysctl_lnb, "I", "LNB");
	
	SYSCTL_ADD_PROC(scl, sol,
					OID_AUTO, "debug", CTLTYPE_INT|CTLFLAG_RW|CTLFLAG_ANYBODY,
					px4, 0, px4_sysctl_debug, "I", "PX4DEBUG");

	SYSCTL_ADD_PROC(scl, sol,
					OID_AUTO, "tsdev_max_packets",CTLTYPE_INT|CTLFLAG_RW|CTLFLAG_ANYBODY,
					px4, 0, px4_sysctl_tsdev_max_packets, "I", "TSDEV-Max Packets");

	SYSCTL_ADD_PROC(scl, sol,
					OID_AUTO, "xfer_packets",CTLTYPE_INT|CTLFLAG_RW|CTLFLAG_ANYBODY,
					px4, 0, px4_sysctl_xfer_packets, "I", "Number of transfer packets from the device" );

	SYSCTL_ADD_PROC(scl, sol,
					OID_AUTO, "usb_max_frames",CTLTYPE_INT|CTLFLAG_RW|CTLFLAG_ANYBODY,
					px4, 0, px4_sysctl_usb_max_frames, "I", "Maximum number of USB frames" );

	SYSCTL_ADD_PROC(scl, sol,
					OID_AUTO, "usb_max_mbufs",CTLTYPE_INT|CTLFLAG_RW|CTLFLAG_ANYBODY,
					px4, 0, px4_sysctl_usb_max_mbufs, "I", "Maximum number of USB mbufs" );

	SYSCTL_ADD_PROC(scl, sol,
					OID_AUTO, "discard_ts_packets",CTLTYPE_INT|CTLFLAG_RW|CTLFLAG_ANYBODY,
					px4, 0, px4_sysctl_discard_ts_packets, "I", "Number of discard ts packets at the start of transfer" );	

	soid = SYSCTL_ADD_NODE(scl, sol,
						   OID_AUTO, "0s", CTLFLAG_RD, 0, "tuner0(ISDB-S)");

	SYSCTL_ADD_PROC(scl, SYSCTL_CHILDREN(soid),
					OID_AUTO, "lnb", CTLTYPE_INT|CTLFLAG_RW|CTLFLAG_ANYBODY,
					&px4->tsdev[0], 0, px4_sysctl_lnb, "I", "LNB");
	SYSCTL_ADD_PROC(scl, SYSCTL_CHILDREN(soid),
					OID_AUTO, "freq", CTLTYPE_INT|CTLFLAG_WR|CTLFLAG_ANYBODY,
					&px4->tsdev[0], 0, px4_sysctl_freq, "I", "channel freq.");
	SYSCTL_ADD_PROC(scl, SYSCTL_CHILDREN(soid),
					OID_AUTO, "signal", CTLTYPE_INT|CTLFLAG_RD,
					&px4->tsdev[0], 0, px4_sysctl_signal, "I", "signal strength");

	soid = SYSCTL_ADD_NODE(scl, sol,
						   OID_AUTO, "1s", CTLFLAG_RD, 0, "tuner1(ISDB-S)");

	SYSCTL_ADD_PROC(scl, SYSCTL_CHILDREN(soid),
					OID_AUTO, "lnb", CTLTYPE_INT|CTLFLAG_RW|CTLFLAG_ANYBODY,
					&px4->tsdev[1], 0, px4_sysctl_lnb, "I", "LNB");
	SYSCTL_ADD_PROC(scl, SYSCTL_CHILDREN(soid),
					OID_AUTO, "freq", CTLTYPE_INT|CTLFLAG_WR|CTLFLAG_ANYBODY,
					&px4->tsdev[1], 0, px4_sysctl_freq, "I", "channel freq.");
	SYSCTL_ADD_PROC(scl, SYSCTL_CHILDREN(soid),
					OID_AUTO, "signal", CTLTYPE_INT|CTLFLAG_RD,
					&px4->tsdev[1], 0, px4_sysctl_signal, "I", "signal strength");

	soid = SYSCTL_ADD_NODE(scl, sol,
						   OID_AUTO, "2t", CTLFLAG_RD, 0, "tuner2(ISDB-T)");

	SYSCTL_ADD_PROC(scl, SYSCTL_CHILDREN(soid),
					OID_AUTO, "lnb", CTLTYPE_INT|CTLFLAG_RW|CTLFLAG_ANYBODY,
					&px4->tsdev[2], 0, px4_sysctl_lnb, "I", "LNB");
	SYSCTL_ADD_PROC(scl, SYSCTL_CHILDREN(soid),
					OID_AUTO, "freq", CTLTYPE_INT|CTLFLAG_WR|CTLFLAG_ANYBODY,
					&px4->tsdev[2], 0, px4_sysctl_freq, "I", "channel freq.");
	SYSCTL_ADD_PROC(scl, SYSCTL_CHILDREN(soid),
					OID_AUTO, "signal", CTLTYPE_INT|CTLFLAG_RD,
					&px4->tsdev[2], 0, px4_sysctl_signal, "I", "signal strength");

	soid = SYSCTL_ADD_NODE(scl, sol,
						   OID_AUTO, "3t", CTLFLAG_RD, 0, "tuner3(ISDB-T)");

	SYSCTL_ADD_PROC(scl, SYSCTL_CHILDREN(soid),
					OID_AUTO, "lnb", CTLTYPE_INT|CTLFLAG_RW|CTLFLAG_ANYBODY,
					&px4->tsdev[3], 0, px4_sysctl_lnb, "I", "LNB");
	SYSCTL_ADD_PROC(scl, SYSCTL_CHILDREN(soid),
					OID_AUTO, "freq", CTLTYPE_INT|CTLFLAG_WR|CTLFLAG_ANYBODY,
					&px4->tsdev[3], 0, px4_sysctl_freq, "I", "channel freq.");
	SYSCTL_ADD_PROC(scl, SYSCTL_CHILDREN(soid),
					OID_AUTO, "signal", CTLTYPE_INT|CTLFLAG_RD,
					&px4->tsdev[3], 0, px4_sysctl_signal, "I", "signal strength");

	return;
}

static int px4_init(struct px4_softc *px4)
{
	int ret = 0;
	int i;

	atomic_set(&px4->ref, 1);
	atomic_set(&px4->avail, 1);

	cv_init(&px4->wait_cv, "px4");
	mutex_init(&px4->wait_mtx);
	sx_init(&px4->xlock, "px4->xlock");
	
	px4->lnb_power_count = 0;
	px4->streaming_count = 0;

	for (i = 0; i < TSDEV_NUM; i++) {
		struct px4_tsdev *tsdev = &px4->tsdev[i];

		sx_init(&tsdev->xlock, "tsdev->xlock");
		tsdev->id = i;
		tsdev->init = false;
		tsdev->open = false;
		tsdev->lnb_power = false;
		atomic_set(&tsdev->streaming, 0);
		atomic_set(&tsdev->ts_packet_read, 0 );
		
		mtx_init(&tsdev->fifo_lock, "tsdev->fifo_lock", NULL, MTX_DEF );
		tsdev->fifo_buffers = NULL;
		tsdev->fifo_index = 0;
		tsdev->fifo_max_index = 0;
		tsdev->fifo_buffer_size = 0;
		tsdev->pending_fifo_size = 0;
		tsdev->ts_packet_count = 0;
		tsdev->sc_fifo_open= NULL;
		px4->stream_context->ptsdev[ i ] = tsdev;
	}
	{
		struct proc *p;
		mtx_init(&px4->stream_context->mtx, "stream_context", NULL, MTX_DEF );
		cv_init(&px4->stream_context->cv, "stream_context");
		px4->stream_context->event = 0;
		
		TAILQ_INIT( &px4->stream_context->stream_buf_list );
		TAILQ_INIT( &px4->stream_context->stream_buf_free );
		for(i=0; i < STREAM_MAX_BUFFERS; i++){
			TAILQ_INSERT_TAIL( &px4->stream_context->stream_buf_free,
							   &px4->stream_context->stream_buf[ i ],
							   entry );

		}
		ret = kproc_create( &stream_thread, px4->stream_context, &p, RFHIGHPID, 0, "stream_thread");
		
		if( !ret ){
			px4->kthread = p;
		}
	}
	
	return ret;
}

static int px4_term(struct px4_softc *px4)
{
	mtx_lock( &px4->stream_context->mtx );
	px4->stream_context->event = -1;
	cv_signal( &px4->stream_context->cv );
	mtx_sleep( px4->kthread, &px4->stream_context->mtx, PWAIT, "stream", 0 );
	mtx_unlock( &px4->stream_context->mtx );
	mtx_destroy( &px4->stream_context->mtx );
	cv_destroy( &px4->stream_context->cv );
	
	cv_destroy( &px4->wait_cv );
	mtx_destroy( &px4->wait_mtx );
	sx_destroy( &px4->xlock);
	
	return 0;
}

static int px4_ref(struct px4_softc *px4)
{
	return atomic_add_return(1, &px4->ref);
}

static int px4_unref(struct px4_softc *px4)
{
	return atomic_sub_return(1, &px4->ref);
}

static int px4_load_config(struct px4_softc *px4)
{
	int ret = 0;
	device_t dev = px4->dev;
	struct it930x_bridge *it930x = &px4->it930x;
	u8 tmp;
	int i;

	ret = it930x_read_reg(it930x, 0x4979, &tmp);
	if (ret) {
		dev_err(dev, "px4_load_config: it930x_read_reg(0x4979) failed.\n");
		return ret;
	} else if (!tmp) {
		dev_warn(dev, "EEPROM is invalid.\n");
		return ret;
	}

	px4->tsdev[0].isdb = ISDB_S;
	px4->tsdev[1].isdb = ISDB_S;
	px4->tsdev[2].isdb = ISDB_T;
	px4->tsdev[3].isdb = ISDB_T;

	it930x->config.input[0].i2c_addr = 0x11;
	it930x->config.input[1].i2c_addr = 0x13;
	it930x->config.input[2].i2c_addr = 0x10;
	it930x->config.input[3].i2c_addr = 0x12;

	for (i = 0; i < TSDEV_NUM; i++) {
		struct it930x_stream_input *input = &it930x->config.input[i];
		struct px4_tsdev *tsdev = &px4->tsdev[i];

		input->enable = true;
		input->is_parallel = false;
		input->port_number = i + 1;
		input->slave_number = i;
		input->i2c_bus = 2;
		input->packet_len = 188;
		input->sync_byte = ((i + 1) << 4) | 0x07;	// 0x17 0x27 0x37 0x47

		tsdev->tc90522.dev = dev;
		tsdev->tc90522.i2c = &it930x->i2c_master[1];
		tsdev->tc90522.i2c_addr = input->i2c_addr;
		tsdev->tc90522.is_secondary = (i % 2) ? true : false;

		switch (tsdev->isdb) {
		case ISDB_S:
			tsdev->t.rt710.dev = dev;
			tsdev->t.rt710.i2c = &tsdev->tc90522.i2c_master;
			tsdev->t.rt710.i2c_addr = 0x7a;
			tsdev->t.rt710.config.xtal = 24000;
			tsdev->t.rt710.config.loop_through = false;
			tsdev->t.rt710.config.clock_out = false;
			tsdev->t.rt710.config.signal_output_mode = RT710_SIGNAL_OUTPUT_DIFFERENTIAL;
			tsdev->t.rt710.config.agc_mode = RT710_AGC_POSITIVE;
			tsdev->t.rt710.config.vga_atten_mode = RT710_VGA_ATTEN_OFF;
			tsdev->t.rt710.config.fine_gain = RT710_FINE_GAIN_3DB;
			tsdev->t.rt710.config.scan_mode = RT710_SCAN_MANUAL;
			break;

		case ISDB_T:
			tsdev->t.r850.dev = dev;
			tsdev->t.r850.i2c = &tsdev->tc90522.i2c_master;
			tsdev->t.r850.i2c_addr = 0x7c;
			tsdev->t.r850.config.xtal = 24000;
			tsdev->t.r850.config.loop_through = (i % 2) ? false : true;
			tsdev->t.r850.config.clock_out = false;
			tsdev->t.r850.config.no_imr_calibration = true;
			tsdev->t.r850.config.no_lpf_calibration = true;
			break;
		}
	}

	it930x->config.input[4].enable = false;
	it930x->config.input[4].port_number = 0;

	return 0;
}

static int px4_set_power(struct px4_softc *px4, bool on)
{
	int ret = 0, i, last_power_count = 0;
	struct it930x_bridge *it930x = &px4->it930x;
	struct px4_multi_device *multi_dev = px4->multi_dev;

	dev_dbg(px4->dev, "px4_set_power: %s\n", on ? "on" : "off");

	if (multi_dev) {
		mutex_lock(&multi_dev->lock);

		dev_dbg(px4->dev, "px4_set_power: power_count: %d\n", multi_dev->power_count);
		last_power_count = multi_dev->power_count;
	}

	if (on) {
		if (multi_dev) {
			if (!multi_dev->power_count) {
				for (i = 0; i < 2; i++) {
					if (!multi_dev->devs[i])
						continue;

					dev_dbg(multi_dev->devs[i]->dev, "px4_set_power: dev %u: gpioh7 low\n", multi_dev->devs[i]->dev_id);

					ret = it930x_write_gpio(&multi_dev->devs[i]->it930x, 7, false);
					if (ret)
						goto exit;

					msleep(75);

					dev_dbg(multi_dev->devs[i]->dev, "px4_set_power: dev %u: gpioh2 high\n", multi_dev->devs[i]->dev_id);

					ret = it930x_write_gpio(&multi_dev->devs[i]->it930x, 2, true);
					if (ret)
						goto exit;

					msleep(20);
				}
			}

			multi_dev->power_count++;
		} else {
			ret = it930x_write_gpio(it930x, 7, false);
			if (ret)
				goto exit;

			msleep(100);

			ret = it930x_write_gpio(it930x, 2, true);
			if (ret)
				goto exit;

			msleep(20);
		}

		for (i = 0; i < TSDEV_NUM; i++) {
			struct px4_tsdev *t = &px4->tsdev[i];

			ret = tc90522_init(&t->tc90522);
			if (ret) {
				dev_err(px4->dev, "px4_set_power: tc90522_init(%d) failed. (ret: %d)\n", i, ret);
				break;
			}

			switch (t->isdb) {
			case ISDB_S:
				ret = rt710_init(&t->t.rt710);
				if (ret)
					dev_err(px4->dev, "px4_set_power: rt710_init(%d) failed. (ret: %d)\n", i, ret);
				break;

			case ISDB_T:
				ret = r850_init(&t->t.r850);
				if (ret)
					dev_err(px4->dev, "px4_set_power: r850_init(%d) failed. (ret: %d)\n", i, ret);
				break;

			default:
				break;
			}

			if (ret)
				break;
		}
	} else {
		for (i = 0; i < TSDEV_NUM; i++) {
			struct px4_tsdev *t = &px4->tsdev[i];

			switch (t->isdb) {
			case ISDB_S:
				rt710_term(&t->t.rt710);
				break;

			case ISDB_T:
				r850_term(&t->t.r850);
				break;

			default:
				break;
			}

			tc90522_term(&t->tc90522);
		}

		if (multi_dev) {
			multi_dev->power_count--;

			if (!multi_dev->power_count) {
				for (i = 0; i < 2; i++) {
					if (!multi_dev->devs[i])
						continue;

					dev_dbg(multi_dev->devs[i]->dev, "px4_set_power: dev %u: gpioh2 low\n", multi_dev->devs[i]->dev_id);
					it930x_write_gpio(&multi_dev->devs[i]->it930x, 2, false);

					dev_dbg(multi_dev->devs[i]->dev, "px4_set_power: dev %u: gpioh7 high\n", multi_dev->devs[i]->dev_id);
					it930x_write_gpio(&multi_dev->devs[i]->it930x, 7, true);
				}
			}
		} else {
			it930x_write_gpio(it930x, 2, false);
			it930x_write_gpio(it930x, 7, true);
		}

		msleep(50);
	}

exit:
	if (ret) {
		if (multi_dev) {
			if (multi_dev->power_count > last_power_count)
				multi_dev->power_count--;
			else if (multi_dev->power_count < last_power_count)
				multi_dev->power_count++;
		}

		dev_err(px4->dev, "px4_set_power: failed.\n");
	} else
		dev_dbg(px4->dev, "px4_set_power: ok\n");

	if (multi_dev)
		mutex_unlock(&multi_dev->lock);

	return ret;
}

static void px4_stream_process(struct px4_tsdev **ptsdev, u8 **buf, u32 *len )
{
#ifdef CONTINUITY_COUNTER_CHECK
	struct px4_softc *px4= container_of( ptsdev[0] , struct px4_softc, tsdev[0]);
#endif
	
	u8 *p = *buf;
	u32 remain = *len;
	struct px4_tsdev *tsdev;
	
	while ( remain ) {
		u32 i;
		bool sync_remain = false;

		for( i = 0; i < TS_SYNC_COUNT; i++ ){
			if (((i + 1) * 188) <= remain) {
				if ((p[i * 188] & 0x8f) != 0x07)
					break;
			} else {
				sync_remain = true;
				break;
			}
		}
		
		if( sync_remain )
			break;
		
		if (i < TS_SYNC_COUNT) {
			p += 1;
			remain -= 1;
			continue;
		}
		
		//pr_debug("px4_stream_process:sync\n");
		while (remain >= 188 && ((p[0] & 0x8f) == 0x07)) {
			u8 id = (p[0] & 0x70) >> 4;

			if (id && id < 5) {
				p[0] = 0x47;
				tsdev= ptsdev[ id - 1 ];
				
				if( atomic_read( &tsdev->ts_packet_read ) ){
					u32 index = tsdev->fifo_index;
					u32 max_buf_size = tsdev->fifo_buffer_size;
					u32 prev_pending_size =tsdev->pending_fifo_size;
					u32 pending_size = prev_pending_size + 188;
					u8 *fifobuf = &tsdev->fifo_buffers[ max_buf_size * index ];
#if 0
					tsdev->discard_ts_packets -= ( tsdev->discard_ts_packets > 0 );
					tsdev->ts_packet_count++;
					
					if( !tsdev->discard_ts_packets ){
						
#ifdef CONTINUITY_COUNTER_CHECK
						u32 pid;
						u32 adaptation_field_control;
						u32 current;
						
						pid  = p[1] << 8;
						pid += p[2];
						pid &= 0x1FFF;
						
						adaptation_field_control = (p[3] & 0x30)>>4;
						current = p[3] & 0xF;
						if( pid != 0x1FFF ){
							if( tsdev->continuity_counter[pid] >= 0 ){
								if( ( adaptation_field_control & 1 ) == 0 ){
									if( current != tsdev->continuity_counter[pid] ){
										dev_dbg( px4->dev, "%s:fifo[%d] ts_packet_count=%lu, adap=0 pid=%x continuity_counter=%d ng header=%02x%02x%02x%02x\n",
												 __FUNCTION__,
												 id-1,
												 tsdev->ts_packet_count,
												 pid,
												 tsdev->continuity_counter[pid],
												 p[0],p[1],p[2],p[3]);
									}
								}
								else {
									if( current != ((tsdev->continuity_counter[pid]+1)&0xF) ){
										dev_dbg( px4->dev, "%s:fifo[%d] ts_packet_count=%lu,pid=%x continuity_counter=%d ng header=%02x%02x%02x%02x\n",
												 __FUNCTION__,
												 id-1,
												 tsdev->ts_packet_count,
												 pid,
												 tsdev->continuity_counter[pid],
												 p[0],p[1],p[2],p[3]);
									}
								}
							}
							tsdev->continuity_counter[pid] = current;
						}
#endif
						memcpy( fifobuf + prev_pending_size, p, 188);
						
						if( pending_size < max_buf_size ){
							tsdev->pending_fifo_size = pending_size;
						}
						else {
							usb_fifo_put_data_buffer( tsdev->sc_fifo_open,
													  fifobuf,
													  max_buf_size );
							
							index++;
							index *= ( index < tsdev->fifo_max_index );
							
							tsdev->fifo_index = index;
							tsdev->pending_fifo_size = 0;
						}
					}
#else
					memcpy( fifobuf + prev_pending_size, p, 188);
					
					if( pending_size < max_buf_size ){
						tsdev->pending_fifo_size = pending_size;
					}
					else {
						usb_fifo_put_data_buffer( tsdev->sc_fifo_open,
												  fifobuf,
												  max_buf_size );
						
						index++;
						index *= ( index < tsdev->fifo_max_index );
							
						tsdev->fifo_index = index;
						tsdev->pending_fifo_size = 0;
					}
#endif
				}
			}
			p += 188;
			remain -= 188;
		}
	}

	*buf = p;
	*len = remain;

	return;
}

static void stream_thread( void *arg )
{
	struct px4_stream_context *stream_context = arg;
	pr_debug("stream_thread:start\n");
	
	for(;;){
		int ev;
		struct stream_buf *stream_buf;
		
		mtx_lock(&stream_context->mtx);
		while( (ev = stream_context->event ) == 0 ){
			//pr_debug("stream_thread:call cv_wait\n");
			cv_wait( &stream_context->cv, &stream_context->mtx);
		}
		stream_context->event = 0;
		mtx_unlock(&stream_context->mtx);

		if( ev == -1 ){
			break;
		}
		else {
			mtx_lock(&stream_context->mtx);
			while( ( stream_buf = TAILQ_FIRST( &stream_context->stream_buf_list ) ) != NULL ){
				TAILQ_REMOVE( &stream_context->stream_buf_list, stream_buf, entry );
				mtx_unlock(&stream_context->mtx);
				{
					u8 *context_remain_buf = stream_context->remain_buf;
					u32 context_remain_len = stream_context->remain_len;
					u32 len = stream_buf->len;
					u32 remain = stream_buf->len;
					u32 offset = 0;
					u8 *p;
					
					if (context_remain_len) {
						if ((context_remain_len + len) >= TS_SYNC_SIZE) {
							u32 write_size;
							
							write_size = TS_SYNC_SIZE - context_remain_len;
							
							memcpy( context_remain_buf + context_remain_len,
									stream_buf->buf,
									write_size );
							context_remain_len = TS_SYNC_SIZE;
							
							px4_stream_process( stream_context->ptsdev, &context_remain_buf, &context_remain_len );
							
							if( !context_remain_len ){
								offset += write_size;
								remain -= write_size;
							}
							
							stream_context->remain_len = 0;
						} else {
							memcpy( context_remain_buf + context_remain_len,
									stream_buf->buf,
									len );
							stream_context->remain_len += len;
							
							mtx_lock(&stream_context->mtx);
							TAILQ_INSERT_TAIL( &stream_context->stream_buf_free, stream_buf, entry );
							continue;
						}
					}
					p = &stream_buf->buf[ offset ];
					
					px4_stream_process( stream_context->ptsdev, &p, &remain );
					
					if (remain) {
						//pr_debug("save remain. remain=%u\n", remain);
						memcpy( stream_context->remain_buf, p, remain );
						stream_context->remain_len = remain;
					}
					
					mtx_lock(&stream_context->mtx);
					TAILQ_INSERT_TAIL( &stream_context->stream_buf_free, stream_buf, entry );
				}
			}
			mtx_unlock(&stream_context->mtx);
		}
	}
	kproc_exit(0);
	
	return;
}

static void px4_stream_buf_getter(void *context, uint8_t **buf, uint32_t *len ){
	
	struct px4_stream_context *stream_context = context;
	struct stream_buf *stream_buf;
	
	mtx_lock(&stream_context->mtx);
	stream_buf = TAILQ_FIRST( &stream_context->stream_buf_free );
	if( stream_buf ){
		TAILQ_REMOVE( &stream_context->stream_buf_free, stream_buf, entry );
		*buf = stream_buf->buf;
		*len = STREAM_BUF_SIZE;
	}
	else {
		stream_context->event = 2;
		cv_signal( &stream_context->cv );
		*buf = NULL;
		//pr_debug("no stream_buf\n");
	}
	mtx_unlock(&stream_context->mtx);
}

static int px4_stream_handler(void *context, uint8_t *buf, u32 len)
{
	struct px4_stream_context *stream_context = context;
	struct stream_buf *stream_buf = container_of( buf, struct stream_buf, buf[ 0 ]);
	
	mtx_lock(&stream_context->mtx);
	{
		if( len ){
			stream_buf->len = len;
			
			TAILQ_INSERT_TAIL( &stream_context->stream_buf_list, stream_buf, entry );
			stream_context->event = 1;
			cv_signal( &stream_context->cv );
		}
		else {
			TAILQ_INSERT_TAIL( &stream_context->stream_buf_free, stream_buf, entry );
		}
	}
	mtx_unlock(&stream_context->mtx);
	
	return 0;
}

static struct tc90522_regbuf tc_init_s[] = {
	{ 0x15, NULL, { 0x00 } },
	{ 0x1d, NULL, { 0x00 } },
	{ 0x04, NULL, { 0x02 } }
};

static struct tc90522_regbuf tc_init_t[] = {
	{ 0xb0, NULL, { 0xa0 } },
	{ 0xb2, NULL, { 0x3d } },
	{ 0xb3, NULL, { 0x25 } },
	{ 0xb4, NULL, { 0x8b } },
	{ 0xb5, NULL, { 0x4b } },
	{ 0xb6, NULL, { 0x3f } },
	{ 0xb7, NULL, { 0xff } },
	{ 0xb8, NULL, { 0xc0 } },
	{ 0x1f, NULL, { 0x00 } },
	{ 0x75, NULL, { 0x00 } }
};

// This function must be called after power on.
static int px4_tsdev_init(struct px4_tsdev *tsdev)
{
	int ret = 0;
	struct px4_softc *px4= container_of(tsdev, struct px4_softc, tsdev[tsdev->id]);
	struct tc90522_demod *tc90522 = &tsdev->tc90522;

	if (tsdev->init)
		// already initialized
		return 0;

	switch (tsdev->isdb) {
	case ISDB_S:
	{
		ret = tc90522_write_multiple_regs(tc90522, tc_init_s, ARRAY_SIZE(tc_init_s));
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_init %d:%u: tc90522_write_multiple_regs(tc_init_s) failed. (ret: %d)\n", px4->dev_idx, tsdev->id, ret);
			break;
		}

		// disable ts pins
		ret = tc90522_enable_ts_pins_s(tc90522, false);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_init %d:%u: tc90522_enable_ts_pins_s(false) failed. (ret: %d)\n", px4->dev_idx, tsdev->id, ret);
			break;
		}

		// wake up
		ret = tc90522_sleep_s(tc90522, false);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_init %d:%u: tc90522_sleep_s(false) failed. (ret: %d)\n", px4->dev_idx, tsdev->id, ret);
			break;
		}

		break;
	}

	case ISDB_T:
	{
		struct r850_system_config sys;

		ret = tc90522_write_multiple_regs(tc90522, tc_init_t, ARRAY_SIZE(tc_init_t));
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_init %d:%u: tc90522_write_multiple_regs(tc_init_t) failed. (ret: %d)\n", px4->dev_idx, tsdev->id, ret);
			break;
		}

		// disable ts pins
		ret = tc90522_enable_ts_pins_t(tc90522, false);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_init %d:%u: tc90522_enable_ts_pins_t(false) failed. (ret: %d)\n", px4->dev_idx, tsdev->id, ret);
			break;
		}

		// wake up
		ret = tc90522_sleep_t(tc90522, false);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_init %d:%u: tc90522_sleep_t(false) failed. (ret: %d)\n", px4->dev_idx, tsdev->id, ret);
			break;
		}

		ret = r850_wakeup(&tsdev->t.r850);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_init %d:%u: r850_wakeup() failed. (ret: %d)\n", px4->dev_idx, tsdev->id, ret);
			break;
		}

		sys.system = R850_SYSTEM_ISDB_T;
		sys.bandwidth = R850_BANDWIDTH_6M;
		sys.if_freq = 4063;

		ret = r850_set_system(&tsdev->t.r850, &sys);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_init %d:%u: r850_set_system() failed. (ret: %d)\n", px4->dev_idx, tsdev->id, ret);
			break;
		}

		break;
	}

	default:
		ret = -EIO;
		break;
	}

	if (!ret)
		tsdev->init = true;

	return ret;
}

static void px4_tsdev_term(struct px4_tsdev *tsdev)
{
	struct tc90522_demod *tc90522 = &tsdev->tc90522;

	if (!tsdev->init)
		return;

	switch (tsdev->isdb) {
	case ISDB_S:
		if (!s_tuner_no_sleep)
			rt710_sleep(&tsdev->t.rt710);

		tc90522_sleep_s(tc90522, true);
		break;

	case ISDB_T:
		r850_sleep(&tsdev->t.r850);
		tc90522_sleep_t(tc90522, true);

		break;

	default:
		break;
	}

	tsdev->init = false;

	return;
}

static int px4_tsdev_set_channel(struct px4_tsdev *tsdev, struct ptx_freq *freq)
{
	struct px4_softc *px4= container_of(tsdev, struct px4_softc, tsdev[tsdev->id]);
	int ret = 0, dev_idx = px4->dev_idx;
	unsigned int tsdev_id = tsdev->id;
	struct tc90522_demod *tc90522 = &tsdev->tc90522;
	u32 real_freq;

	dev_dbg(px4->dev, "px4_tsdev_set_channel %d:%u: freq_no: %d, slot: %d\n", dev_idx, tsdev_id, freq->freq_no, freq->slot);

	switch (tsdev->isdb) {
	case ISDB_S:
	{
		int i;
		struct rt710_tuner *rt710 = &tsdev->t.rt710;
		bool tuner_locked, demod_locked;
		s32 ss = 0;
		u16 tsid, tsid2;

		if (freq->freq_no < 0) {
			ret = -EINVAL;
			break;
		} else if (freq->freq_no < 12) {
			if (freq->slot >= 8) {
				ret = -EINVAL;
				break;
			}
			real_freq = 1049480 + (38360 * freq->freq_no);
		} else if (freq->freq_no < 24) {
			real_freq = 1613000 + (40000 * (freq->freq_no - 12));
		} else {
			ret = -EINVAL;
			break;
		}

		// set frequency

		ret = tc90522_set_agc_s(tc90522, false);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_set_agc_s(false) failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}
		ret = tc90522_write_reg(tc90522, 0x8e, 0x06/*0x02*/);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_write_reg(0x8e, 0x06) failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}
		ret = tc90522_write_reg(tc90522, 0xa3, 0xf7);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_write_reg(0xa3, 0xf7) failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}

		ret = rt710_set_params(rt710, real_freq, 28860, 4);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: rt710_set_params(%u, 28860, 4) failed. (ret: %d)\n", dev_idx, tsdev_id, real_freq, ret);
			break;
		}

		i = 50;
		while (i--) {
			ret = rt710_is_pll_locked(rt710, &tuner_locked);
			if (!ret && tuner_locked)
				break;

			msleep(10);
		}

		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: rt710_is_pll_locked() failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		} else if (!tuner_locked) {
			// PLL error
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: PLL is NOT locked.\n", dev_idx, tsdev_id);
			ret = -EAGAIN;
			break;
		}

		rt710_get_rf_signal_strength(rt710, &ss);

		dev_dbg(px4->dev, "px4_tsdev_set_channel %d:%u: PLL is locked. count: %d, signal strength: %ddBm\n", dev_idx, tsdev_id, i, ss);

		ret = tc90522_set_agc_s(tc90522, true);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_set_agc_s(true) failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}

		// check lock

		i = 300;
		while (i--) {
			ret = tc90522_is_signal_locked_s(tc90522, &demod_locked);
			if (!ret && demod_locked)
				break;

			msleep(10);
		}

		if (ret) {
			dev_dbg(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_is_signal_locked_s() failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}

		dev_dbg(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_is_signal_locked_s() locked: %d, count: %d\n", dev_idx, tsdev_id, demod_locked, i);

		if (!demod_locked) {
			ret = -EAGAIN;
			break;
		}

		// set slot

		i = 100;
		while (i--) {
			ret = tc90522_tmcc_get_tsid_s(tc90522, freq->slot, &tsid);
			if ((!ret && tsid) || ret == -EINVAL)
				break;

			msleep(10);
		}

		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_tmcc_get_tsid_s() failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}

		dev_dbg(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_tmcc_get_tsid_s() tsid: 0x%04x, count: %d\n", dev_idx, tsdev_id, tsid, i);

		if (!tsid) {
			ret = -EAGAIN;
			break;
		}

		ret = tc90522_set_tsid_s(tc90522, tsid);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_set_tsid_s(0x%x) failed. (ret: %d)\n", dev_idx, tsdev_id, tsid, ret);
			break;
		}

		// check slot

		i = 100;
		while(i--) {
			ret = tc90522_get_tsid_s(tc90522, &tsid2);
			if (!ret && tsid2 == tsid)
				break;

			msleep(10);
		}

		dev_dbg(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_get_tsid_s() tsid2: 0x%04x, count: %d\n", dev_idx, tsdev_id, tsid2, i);

		if (tsid2 != tsid) {
			ret = -EAGAIN;
			break;
		}

		break;
	}

	case ISDB_T:
	{
		int i;
		bool tuner_locked, demod_locked;

		if ((freq->freq_no >= 3 && freq->freq_no <= 12) || (freq->freq_no >= 22 && freq->freq_no <= 62)) {
			// CATV C13-C22ch, C23-63ch
			real_freq = 93143 + freq->freq_no * 6000 + freq->slot/* addfreq */;

			if (freq->freq_no == 12)
				real_freq += 2000;
		} else if (freq->freq_no >= 63 && freq->freq_no <= 102) {
			// UHF 13-52ch
			real_freq = 95143 + freq->freq_no * 6000 + freq->slot/* addfreq */;
		} else {
			// Unknown channel
			ret = -EINVAL;
			break;
		}

		// set frequency

		ret = tc90522_write_reg(tc90522, 0x47, 0x30);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_write_reg(0x47, 0x30) failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}

		ret = tc90522_set_agc_t(tc90522, false);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_set_agc_t(false) failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}

		ret = tc90522_write_reg(tc90522, 0x76, 0x0c);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_write_reg(0x76, 0x0c) failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}

		ret = r850_set_frequency(&tsdev->t.r850, real_freq);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: r850_set_frequency(%u) failed. (ret: %d)\n", dev_idx, tsdev_id, real_freq, ret);
			break;
		}

		i = 50;
		while (i--) {
			ret = r850_is_pll_locked(&tsdev->t.r850, &tuner_locked);
			if (!ret && tuner_locked)
				break;

			msleep(10);
		}

		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: r850_is_pll_locked() failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		} else if (!tuner_locked) {
			// PLL error
			dev_dbg(px4->dev, "px4_tsdev_set_channel %d:%u: PLL is NOT locked.\n", dev_idx, tsdev_id);
			ret = -EAGAIN;
			break;
		}

		dev_dbg(px4->dev, "px4_tsdev_set_channel %d:%u: PLL is locked. count: %d\n", dev_idx, tsdev_id, i);

		ret = tc90522_set_agc_t(tc90522, true);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_set_agc_t(true) failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}

		ret = tc90522_write_reg(tc90522, 0x71, 0x21);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_write_reg(0x71, 0x21) failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}
		ret = tc90522_write_reg(tc90522, 0x72, 0x25);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_write_reg(0x72, 0x25) failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}
		ret = tc90522_write_reg(tc90522, 0x75, 0x08);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_write_reg(0x75, 0x08) failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}

		// check lock

		i = 300;
		while (i--) {
			ret = tc90522_is_signal_locked_t(tc90522, &demod_locked);
			if (!ret && demod_locked)
				break;

			msleep(10);
		}

		if (ret) {
			dev_err(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_is_signal_locked_t() failed. (ret: %d)\n", dev_idx, tsdev_id, ret);
			break;
		}

		dev_dbg(px4->dev, "px4_tsdev_set_channel %d:%u: tc90522_is_signal_locked_t() locked: %d, count: %d\n", dev_idx, tsdev_id, demod_locked, i);

		if (!demod_locked) {
			ret = -EAGAIN;
			break;
		}

		if (i > 265)
			msleep((i - 265) * 10);

		break;
	}

	default:
		ret = -EIO;
		break;
	}

	if (!ret)
		dev_dbg(px4->dev, "px4_tsdev_set_channel %d:%u: succeeded.\n", dev_idx, tsdev_id);
	else
		dev_dbg(px4->dev, "px4_tsdev_set_channel %d:%u: failed. (ret: %d)\n", dev_idx, tsdev_id, ret);

	return ret;
}

static int px4_tsdev_start_streaming(struct px4_tsdev *tsdev)
{
	int ret = 0;
	struct px4_softc *px4= container_of(tsdev, struct px4_softc, tsdev[tsdev->id]);
	struct it930x_bus *bus = &px4->it930x.bus;
	struct tc90522_demod *tc90522 = &tsdev->tc90522;
	unsigned int streaming_count;

	if (atomic_read(&tsdev->streaming))
		// already started
		return 0;

	atomic_set(&tsdev->streaming, 1);

#if defined(__FreeBSD__)
	sx_xlock( &px4->xlock );
#else
	mutex_lock(&px4->lock);
#endif

	if (!px4->streaming_count) {
		
		dev_dbg(px4->dev, "px4_tsdev_start_streaming %d:%u: stream_usb_frames=%u, stream_usb_bufsize=%u\n", px4->dev_idx, tsdev->id, bus->usb.stream_usb_frames, bus->usb.stream_usb_bufsize );

#if 0
		ret = it930x_purge_psb(&px4->it930x, 2000);  // timeout is ignored
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_start_streaming %d:%u: it930x_purge_psb() failed. (ret: %d)\n", px4->dev_idx, tsdev->id, ret);
			goto fail;
		}
#endif
	}

	switch (tsdev->isdb) {
	case ISDB_S:
		// enable ts pins
		ret = tc90522_enable_ts_pins_s(tc90522, true);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_start_streaming %d:%u: tc90522_enable_ts_pins_s(true) failed. (ret: %d)\n", px4->dev_idx, tsdev->id, ret);

			// disable ts pins
			tc90522_enable_ts_pins_s(tc90522, false);
		}
		break;

	case ISDB_T:
		// enable ts pins
		ret = tc90522_enable_ts_pins_t(tc90522, true);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_start_streaming %d:%u: tc90522_enable_ts_pins_t(true) failed. (ret: %d)\n", px4->dev_idx, tsdev->id, ret);

			// disable ts pins
			tc90522_enable_ts_pins_t(tc90522, false);
		}
		break;

	default:
		ret = -EIO;
		break;
	}

	if (ret)
		goto fail;

	if (!px4->streaming_count) {
		px4->stream_context->remain_len = 0;
		px4->stream_context->sync_flag = 0;
		px4->stream_context->resync_count = 0;
		dev_dbg(px4->dev, "px4_tsdev_start_streaming %d:%u: starting...\n", px4->dev_idx, tsdev->id);
#if defined(__FreeBSD__)
		ret = it930x_bus_start_streaming(bus, px4_stream_buf_getter, px4_stream_handler, px4->stream_context);
#else
		ret = it930x_bus_start_streaming(bus, px4_stream_handler, px4->stream_context);
#endif
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_start_streaming %d:%u: it930x_bus_start_streaming() failed. (ret: %d)\n", px4->dev_idx, tsdev->id, ret);
			goto fail_after_ringbuffer;
		}
		mtx_lock( &bus->usb.xfer_mtx );
		usbd_transfer_start( bus->usb.transfer[ IT930X_BUS_STREAM_RD ] );
		mtx_unlock( &bus->usb.xfer_mtx );
	}
	
	px4->streaming_count++;
	streaming_count = px4->streaming_count;

#if defined(__FreeBSD__)
	sx_xunlock( &px4->xlock );
#else
	mutex_unlock(&px4->lock);
#endif
	
	dev_dbg(px4->dev, "px4_tsdev_start_streaming %d:%u: streaming_count: %u\n", px4->dev_idx, tsdev->id, streaming_count);
	return ret;

fail_after_ringbuffer:
	//	ringbuffer_stop(tsdev->ringbuf);
fail:
#if defined(__FreeBSD__)
	sx_xunlock( &px4->xlock );
#else
	mutex_unlock(&px4->lock);
#endif
	atomic_set(&tsdev->streaming, 0);

	dev_err(px4->dev, "px4_tsdev_start_streaming %d:%u: failed. (ret: %d)\n", px4->dev_idx, tsdev->id, ret);

	return ret;
}

static int px4_tsdev_stop_streaming(struct px4_tsdev *tsdev, bool avail)
{
	int ret = 0;
	struct px4_softc *px4= container_of(tsdev, struct px4_softc, tsdev[tsdev->id]);
	struct tc90522_demod *tc90522 = &tsdev->tc90522;
	struct it930x_bus *bus = &px4->it930x.bus;
	unsigned int streaming_count;

	if (!atomic_read(&tsdev->streaming))
		// already stopped
		return 0;

	atomic_set(&tsdev->streaming, 0);

#if defined(__FreeBSD__)
	sx_xlock(&px4->xlock);
#else
	mutex_lock(&px4->lock);
#endif

	px4->streaming_count--;
	if (!px4->streaming_count) {
		dev_dbg(px4->dev, "px4_tsdev_stop_streaming %d:%u: stopping...\n", px4->dev_idx, tsdev->id);
		it930x_bus_stop_streaming(&px4->it930x.bus);

		mtx_lock( &bus->usb.xfer_mtx );
		usbd_transfer_stop( bus->usb.transfer[ IT930X_BUS_STREAM_RD ] );
		mtx_unlock( &bus->usb.xfer_mtx );
		
		TAILQ_INIT( &px4->stream_context->stream_buf_list );
		TAILQ_INIT( &px4->stream_context->stream_buf_free );
		
		int i;
		for(i=0; i < STREAM_MAX_BUFFERS; i++){
			TAILQ_INSERT_TAIL( &px4->stream_context->stream_buf_free,
							   &px4->stream_context->stream_buf[ i ],
							   entry );

		}
	}
	streaming_count = px4->streaming_count;

#if defined(__FreeBSD__)
	if (!avail){
		sx_xunlock( &px4->xlock);
		return 0;
	}
#else
	mutex_unlock(&px4->lock);

	ringbuffer_stop(tsdev->ringbuf);
	
	if (!avail)
		return 0;
#endif
	switch (tsdev->isdb) {
	case ISDB_S:
		// disable ts pins
		ret = tc90522_enable_ts_pins_s(tc90522, false);
		break;

	case ISDB_T:
		// disable ts pins
		ret = tc90522_enable_ts_pins_t(tc90522, false);
		break;

	default:
		ret = -EIO;
		break;
	}
#if defined(__FreeBSD__)
	sx_xunlock( &px4->xlock);
#endif

	dev_dbg(px4->dev, "px4_tsdev_stop_streaming %d:%u: streaming_count: %u\n", px4->dev_idx, tsdev->id, streaming_count);

	return ret;
}

static int px4_tsdev_get_cn(struct px4_tsdev *tsdev, u32 *cn)
{
	int ret = 0;
	struct tc90522_demod *tc90522 = &tsdev->tc90522;

	switch (tsdev->isdb) {
	case ISDB_S:
		ret = tc90522_get_cn_s(tc90522, (u16 *)cn);
		break;

	case ISDB_T:
		ret = tc90522_get_cndat_t(tc90522, cn);
		break;

	default:
		ret = -EIO;
		break;
	}

	return ret;
}

static int px4_tsdev_set_lnb_power(struct px4_tsdev *tsdev, bool enable)
{
	int ret = 0;
	struct px4_softc *px4= container_of(tsdev, struct px4_softc, tsdev[tsdev->id]);

	if ((tsdev->lnb_power && enable) || (!tsdev->lnb_power && !enable))
		return 0;

#if defined(__FreeBSD__)
	sx_xlock( &px4->xlock );
#else
	mutex_lock(&px4->lock);
#endif

	if (enable) {
		if (!px4->lnb_power_count)
			ret = it930x_write_gpio(&px4->it930x, 11, true);

		px4->lnb_power_count++;
	} else {
		if (px4->lnb_power_count == 1)
			ret = it930x_write_gpio(&px4->it930x, 11, false);

		px4->lnb_power_count--;
	}

#if defined(__FreeBSD__)
	sx_xunlock( &px4->xlock);
#else
	mutex_unlock(&px4->lock);
#endif

	tsdev->lnb_power = enable;

	return ret;
}

static int px4_sysctl_debug(SYSCTL_HANDLER_ARGS)
{
	int val = px4_debug;
	int error;

	error = sysctl_handle_int( oidp, &val, 0, req );
	if( error ){
		return error;
	}
	// debug on/off
	if( val != px4_debug){
		px4_debug= val;
	}
	
	return error;
}

static int px4_sysctl_tsdev_max_packets(SYSCTL_HANDLER_ARGS)
{
	int val = tsdev_max_packets;
	int error;

	error = sysctl_handle_int( oidp, &val, 0, req );
	if( error ){
		return error;
	}
	if( val != tsdev_max_packets){
		tsdev_max_packets= val;
	}
	
	return error;
}

static int px4_sysctl_xfer_packets(SYSCTL_HANDLER_ARGS)
{
	int val = xfer_packets;
	int error;

	error = sysctl_handle_int( oidp, &val, 0, req );
	if( error ){
		return error;
	}
	if( val != xfer_packets){
		xfer_packets = val;
	}
	
	return error;
}

static int px4_sysctl_usb_max_frames(SYSCTL_HANDLER_ARGS)
{
	int val = usb_max_frames;
	int error;

	error = sysctl_handle_int( oidp, &val, 0, req );
	if( error ){
		return error;
	}
	if( val != usb_max_frames){
		usb_max_frames = val;
	}
	
	return error;
}

static int px4_sysctl_usb_max_mbufs(SYSCTL_HANDLER_ARGS)
{
	int val = usb_max_mbufs;
	int error;

	error = sysctl_handle_int( oidp, &val, 0, req );
	if( error ){
		return error;
	}
	if( val != usb_max_mbufs){
		usb_max_mbufs = val;
	}
	
	return error;
}

static int px4_sysctl_discard_ts_packets(SYSCTL_HANDLER_ARGS)
{
	int val = discard_ts_packets;
	int error;

	error = sysctl_handle_int( oidp, &val, 0, req );
	if( error ){
		return error;
	}
	if( val != discard_ts_packets){
		discard_ts_packets = val;
	}
	
	return error;
}

static int px4_sysctl_lnb(SYSCTL_HANDLER_ARGS)
{
	struct px4_tsdev *tsdev = (struct px4_tsdev *)arg1;
	struct px4_softc *px4= container_of(tsdev, struct px4_softc, tsdev[tsdev->id]);
	int avail;
	int lnb = (tsdev->lnb_power) ? 2:0;
	bool b;
	int error;

	error = sysctl_handle_int( oidp, &lnb, 0, req );
	if( error || !req->newptr){
		return error;
	}
	
	if( tsdev->isdb != ISDB_S ){
		return -EINVAL;
	}

	avail = atomic_read(&px4->avail);
	if( !avail ){
		return -EIO;
	}

	if( lnb == 0 ){
		// 0V
		b = false;
	}
	else if( lnb == 2 ){
		// 15V
		b = true;
	}
	else {
		dev_dbg(px4->dev, "px4_sysctl_lnb:EINVAL\n");	
		return -EINVAL;
	}
	
	sx_xlock( &tsdev->xlock);
	error= px4_tsdev_set_lnb_power(tsdev, b);
	sx_xunlock(&tsdev->xlock);
	if( error ){
		dev_dbg(px4->dev, "px4_sysctl_lnb:error=%d\n", error);
	}

	return error;
}

static int px4_sysctl_freq(SYSCTL_HANDLER_ARGS)
{
	struct px4_tsdev *tsdev = (struct px4_tsdev *)arg1;
	struct px4_softc *px4= container_of(tsdev, struct px4_softc, tsdev[tsdev->id]);
	int avail;
	int val = tsdev->freq;
	struct ptx_freq freq;
	int error;

	
	error = sysctl_handle_int( oidp, &val, 0, req );
	if( error || !req->newptr){
		return error;
	}
	avail = atomic_read(&px4->avail);
	
	if( !avail ){
		dev_dbg(px4->dev, "px4_sysctl_freq:not avail\n");	
		return -EIO;
	}

	tsdev->freq= val;
	
	freq.freq_no = (val & 0xffff);
	freq.slot = ((val >> 16)& 0xffff);

#if defined(__FreeBSD__)
	sx_xlock( &tsdev->xlock );
	//sx_xlock( &px4->xlock );
#endif
	error = px4_tsdev_set_channel( tsdev, &freq );
#if defined(__FreeBSD__)
	//sx_xunlock( &px4->xlock );
	sx_xunlock(&tsdev->xlock );
#endif
	
	return error;
}

static int px4_sysctl_signal(SYSCTL_HANDLER_ARGS)
{
	struct px4_tsdev *tsdev = (struct px4_tsdev *)arg1;
	struct px4_softc *px4= container_of(tsdev, struct px4_softc, tsdev[tsdev->id]);
	int avail;
	int cn;
	int error;

	avail = atomic_read(&px4->avail);
	
	if( !avail || !tsdev->open){
		return -EIO;
	}
	if(req->newptr){
		return EPERM;
	}

#if defined(__FreeBSD__)
	sx_xlock( &tsdev->xlock );
	//sx_xlock( &px4->xlock );
#endif
	error = px4_tsdev_get_cn(tsdev, (u32 *)&cn);
#if defined(__FreeBSD__)
	//sx_xunlock( &px4->xlock );
	sx_xunlock(&tsdev->xlock );
#endif
	if( error ){
		return error;
	}
	
	return sysctl_handle_int(oidp, NULL, cn, req);
}

static struct tc90522_regbuf tc_init_s0[] = {
	{ 0x07, NULL, { 0x31 } },
	{ 0x08, NULL, { 0x77 } }
};

static struct tc90522_regbuf tc_init_t0[] = {
	{ 0x0e, NULL, { 0x77 } },
	{ 0x0f, NULL, { 0x13 } }
};

static int px4_tsdev_open(struct usb_fifo *fifo, int fflags)
{
	struct px4_tsdev *tsdev =usb_fifo_softc( fifo );
	struct px4_softc *px4= container_of(tsdev, struct px4_softc, tsdev[tsdev->id]);
	//struct it930x_bus *bus = &px4->it930x.bus;
	
	int ret = 0, ref;
	int dev_idx = device_get_unit(px4->dev);
	unsigned int tsdev_id = tsdev->id;
	int error;
	unsigned int bufsize;

#if defined(__FreeBSD__)
	//sx_xlock(&px4->xlock);
	sx_xlock(&glock);
#else
	mutex_lock(&glock);
#endif	
	if (!atomic_read(&px4->avail)) {
		// not available
#if defined(__FreeBSD__)
		sx_xunlock(&glock);
		//sx_xunlock(&px4->xlock);
#else
		mutex_unlock(&glock);
#endif
		return -EIO;
	}
	
	tsdev = &px4->tsdev[tsdev_id];
	
#if defined(__FreeBSD__)
	sx_xlock(&tsdev->xlock);
	sx_xunlock(&glock);
#else
	mutex_lock(&tsdev->lock);
	mutex_unlock(&glock);
#endif
	
	if (tsdev->open) {
		// already used by another
		ret = -EALREADY;
#if defined(__FreeBSD__)
		sx_xunlock(&tsdev->xlock);
#else
		mutex_unlock(&tsdev->lock);
#endif

		goto fail;
	}

#if defined(__FreeBSD__)
	sx_xlock(&px4->xlock);
#else
	mutex_lock(&px4->lock);
#endif
	
	ref = px4_ref(px4);
	if (ref <= 1) {
		ret = -ECANCELED;
#if defined(__FreeBSD__)
		sx_xunlock(&px4->xlock);
		sx_xunlock(&tsdev->xlock);
#else
		mutex_unlock(&px4->lock);
		mutex_unlock(&tsdev->lock);
#endif
		goto fail;
	}

	dev_dbg(px4->dev, "px4_tsdev_open %d:%u: ref count: %d\n", dev_idx, tsdev_id, ref);

	if (ref == 2) {
		int i;

		ret = px4_set_power(px4, true);
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_open %d:%u: px4_set_power(true) failed.\n", dev_idx, tsdev_id);
			goto fail_after_ref;
		}

		for (i = 0; i < TSDEV_NUM; i++) {
			struct px4_tsdev *t = &px4->tsdev[i];

			if (i == tsdev->id)
				continue;

			if (!t->open) {
				switch (t->isdb) {
				case ISDB_S:
					if (!s_tuner_no_sleep) {
						ret = rt710_sleep(&t->t.rt710);
						if (ret) {
							dev_err(px4->dev, "px4_tsdev_open %d:%u: rt710_sleep(%d) failed. (ret: %d)\n", dev_idx, tsdev_id, i, ret);
							break;
						}
					}

					ret = tc90522_sleep_s(&t->tc90522, true);
					if (ret) {
						dev_err(px4->dev, "px4_tsdev_open %d:%u: tc90522_sleep_s(%d, true) failed. (ret: %d)\n", dev_idx, tsdev_id, i, ret);
						break;
					}

					break;

				case ISDB_T:
					ret = r850_sleep(&t->t.r850);
					if (ret) {
						dev_err(px4->dev, "px4_tsdev_open %d:%u: rt850_sleep(%d) failed. (ret: %d)\n", dev_idx, tsdev_id, i, ret);
						break;
					}

					ret = tc90522_sleep_t(&t->tc90522, true);
					if (ret) {
						dev_err(px4->dev, "px4_tsdev_open %d:%u: tc90522_sleep_t(%d, true) failed. (ret: %d)\n", dev_idx, tsdev_id, i, ret);
						break;
					}

					break;

				default:
					break;
				}
			}

			if (ret)
				break;
		}

		if (i < TSDEV_NUM)
			goto fail_after_power;
	}

	ret = px4_tsdev_init(tsdev);
	if (ret) {
		dev_err(px4->dev, "px4_tsdev_open %d:%u: px4_tsdev_init() failed.\n", dev_idx, tsdev_id);
		goto fail_after_power;
	}

	if (ref == 2) {
		// S0
		ret = tc90522_write_multiple_regs(&px4->tsdev[0].tc90522, tc_init_s0, ARRAY_SIZE(tc_init_s0));
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_open %d:%u: tc90522_write_multiple_regs(tc_init_s0) failed.\n", dev_idx, tsdev_id);
			goto fail_after_power;
		}

		// T0
		ret = tc90522_write_multiple_regs(&px4->tsdev[2].tc90522, tc_init_t0, ARRAY_SIZE(tc_init_t0));
		if (ret) {
			dev_err(px4->dev, "px4_tsdev_open %d:%u: tc90522_write_multiple_regs(tc_init_t0) failed.\n", dev_idx, tsdev_id);
			goto fail_after_power;
		}
	}

	error = usb_fifo_alloc_buffer( fifo,
								   1,   // dummy
								   usb_max_mbufs );
	
	if ( error ) {
		ret= ENOMEM;
		goto fail_after_power;
	}
	
	bufsize = 188 * tsdev_max_packets * usb_max_mbufs;
	tsdev->fifo_buffers = kmalloc( sizeof(u8) * bufsize, GFP_ATOMIC);
	if ( tsdev->fifo_buffers == NULL ){
		usb_fifo_free_buffer( fifo );
		ret= ENOMEM;
		goto fail_after_power;
	}
	tsdev->fifo_buffer_size = 188 * tsdev_max_packets;
	tsdev->fifo_index = 0;
	tsdev->pending_fifo_size = 0;
	tsdev->fifo_max_index = usb_max_mbufs;
	tsdev->discard_ts_packets = discard_ts_packets;

	dev_info( px4->dev,"px4_tsdev_open %d:%u: usb_max_mbufs=%u, fifo_bufsize per mbufs=%u, tsdev_max_packets=%u\n", dev_idx, tsdev_id, usb_max_mbufs, bufsize, tsdev_max_packets );

#ifdef  CONTINUITY_COUNTER_CHECK
	{
		u32 count;
		for( count = 0; count < 0x2000; count++){
			tsdev->continuity_counter[count] = -1;
		}
	}
#endif
	
	tsdev->sc_fifo_open = fifo;
	
	tsdev->open = true;

#if defined(__FreeBSD__)
	
/*
	dev_dbg( px4->dev, "px4_tsdev_open:start_streaming\n" );

	error = px4_tsdev_start_streaming( tsdev );
	  
	if(error){
		dev_dbg( px4->dev, "tsdev_id=%d error=%d", tsdev->id, error );
	}
*/	
	sx_xunlock(&px4->xlock);
	sx_xunlock(&tsdev->xlock);
#else
	mutex_unlock(&px4->lock);
	mutex_unlock(&tsdev->lock);
#endif


	dev_dbg(px4->dev, "px4_tsdev_open %d:%u: ok\n", dev_idx, tsdev_id);

	return 0;

fail_after_power:
	if (ref == 2)
		px4_set_power(px4, false);
fail_after_ref:
	px4_unref(px4);
#if defined(__FreeBSD__)
	sx_xunlock(&px4->xlock);
	sx_xunlock(&tsdev->xlock);
#else
	mutex_unlock(&px4->lock);
	mutex_unlock(&tsdev->lock);
#endif
fail:
#if defined(__FreeBSD__)
	//sx_xunlock(&px4->xlock);
#endif
	dev_err(px4->dev, "px4_tsdev_open %d:%u: failed. (ret: %d)\n", dev_idx, tsdev_id, ret);

	return ret;
}


static void px4_tsdev_release(struct usb_fifo *fifo, int fflags)
{
	int avail, ref;
	struct px4_softc *px4;
	struct px4_tsdev *tsdev =usb_fifo_softc( fifo );
	
	if (!tsdev) {
		pr_err("px4_tsdev_release: tsdev is NULL.\n");
		return;
	}

	px4 = container_of(tsdev, struct px4_softc, tsdev[tsdev->id]);
	avail = atomic_read(&px4->avail);

	dev_dbg(px4->dev, "px4_tsdev_release %d:%u: avail: %d\n", px4->dev_idx, tsdev->id, avail);

#if defined(__FreeBSD__)
	sx_xlock(&tsdev->xlock);
#else
	mutex_lock(&tsdev->lock);
#endif

	if (atomic_read(&tsdev->streaming)){
		dev_dbg(px4->dev, "px4_tsdev_release %d:%u: call px4_tsdev_stop_streaming\n", px4->dev_idx, tsdev->id);
		px4_tsdev_stop_streaming(tsdev, (avail) ? true : false);
	}
	
	if (avail && tsdev->isdb == ISDB_S)
		px4_tsdev_set_lnb_power(tsdev, false);
	
	if ( tsdev->open ){
		tsdev->sc_fifo_open = NULL;
		usb_fifo_free_buffer( fifo );

		if( tsdev->fifo_buffers != NULL){
			kfree( tsdev->fifo_buffers );
			tsdev->fifo_buffers = NULL;
		}
		tsdev->fifo_index = 0;
		tsdev->fifo_max_index = 0;
		tsdev->fifo_buffer_size = 0;
		tsdev->pending_fifo_size = 0;
		tsdev->ts_packet_count = 0;
	}
	
#if defined(__FreeBSD__)
	sx_xlock(&px4->xlock);
#else
 	mutex_lock(&px4->lock);
#endif	
	
	if (avail)
		px4_tsdev_term(tsdev);

	ref = px4_unref(px4);
	if (avail && ref <= 1)
		px4_set_power(px4, false);

#if defined(__FreeBSD__)
	sx_xunlock(&px4->xlock);
#else
	mutex_unlock(&px4->lock);
#endif
	
	tsdev->open = false;
	
#if defined(__FreeBSD__)
	sx_xunlock(&tsdev->xlock);
#else
	mutex_unlock(&tsdev->lock);
#endif
	
	mutex_lock( &px4->wait_mtx );
	cv_signal( &px4->wait_cv );
	mutex_unlock( &px4->wait_mtx );
	//wake_up(&px4->wait);
  
	dev_dbg(px4->dev, "px4_tsdev_release %d:%u: ok. ref count: %d\n", px4->dev_idx, tsdev->id, ref);

	return;
}

static void px4_tsdev_start_read( struct usb_fifo *fifo )
{
	struct px4_tsdev *tsdev =usb_fifo_softc( fifo );
	struct px4_softc *px4= container_of(tsdev, struct px4_softc, tsdev[tsdev->id]);

	if( !atomic_read(&tsdev->ts_packet_read ) ){
		dev_dbg(px4->dev, "px4_tsdev_start_read %d:%u:\n", px4->dev_idx, tsdev->id);
		tsdev->discard_ts_packets = discard_ts_packets;
	}
	atomic_set(&tsdev->ts_packet_read, 1 );
	
	sx_xlock( &tsdev->xlock );
	{
		int error;
		dev_dbg( px4->dev, "px4_tsdev_open:start_streaming\n" );
		error = px4_tsdev_start_streaming( tsdev );
		if(error){
			dev_dbg( px4->dev, "tsdev_id=%d error=%d", tsdev->id, error );
		}
	}
	sx_xunlock( &tsdev->xlock );
	
	return;
}

static void px4_tsdev_stop_read( struct usb_fifo *fifo )
{
	struct px4_tsdev *tsdev =usb_fifo_softc( fifo );
	struct px4_softc *px4= container_of(tsdev, struct px4_softc, tsdev[tsdev->id]);

	if( atomic_read(&tsdev->ts_packet_read ) ){
		dev_dbg(px4->dev, "px4_tsdev_stop_read %d:%u:\n", px4->dev_idx, tsdev->id);
	}
	atomic_set(&tsdev->ts_packet_read, 0 );
	
	return;
}

#if 0
static int px4_tsdev_ioctl(struct usb_fifo *fifo, u_long cmd, void *data, int fflags)
{
	int ret = -EIO;
	struct px4_tsdev *tsdev =usb_fifo_softc( fifo );
	struct px4_softc *px4= container_of(tsdev, struct px4_softc, tsdev[tsdev->id]);
	int avail;
	int dev_idx;
	unsigned int tsdev_id;

	avail = atomic_read(&px4->avail);

	dev_idx = px4->dev_idx;
	tsdev_id = tsdev->id;

	mutex_lock(&tsdev->lock);

	switch (cmd) {
	case PTX_SET_CHANNEL:
	{
		struct ptx_freq *freq;

		dev_dbg(px4->dev, "px4_tsdev_ioctl %d:%u: PTX_SET_CHANNEL\n", dev_idx, tsdev_id);

		if (!avail) {
			ret = -EIO;
			break;
		}
		freq = (struct ptx_freq *)data;

		ret = px4_tsdev_set_channel(tsdev, freq);
		break;
	}

	case PTX_START_STREAMING:
		dev_dbg(px4->dev, "px4_tsdev_ioctl %d:%u: PTX_START_STREAMING\n", dev_idx, tsdev_id);

		if (!avail) {
			ret = -EIO;
			break;
		}

		ret = px4_tsdev_start_streaming(tsdev);
		break;

	case PTX_STOP_STREAMING:
		dev_dbg(px4->dev, "px4_tsdev_ioctl %d:%u: PTX_STOP_STREAMING\n", dev_idx, tsdev_id);
		ret = px4_tsdev_stop_streaming(tsdev, (avail) ? true : false);
		break;

	case PTX_GET_CNR:
	{
		int cn = 0;

		dev_dbg(px4->dev, "px4_tsdev_ioctl %d:%u: PTX_GET_CNR\n", dev_idx, tsdev_id);

		if (!avail) {
			ret = -EIO;
			break;
		}

		ret = px4_tsdev_get_cn(tsdev, (u32 *)&cn);
		if (!ret)
			*((int*)data)= cn;

		break;
	}

	case PTX_ENABLE_LNB_POWER:
	{
		int lnb;
		bool b;

		lnb = *(int*)data;

		dev_dbg(px4->dev, "px4_tsdev_ioctl %d:%u: PTX_ENABLE_LNB_POWER lnb: %d\n", dev_idx, tsdev_id, lnb);

		if (tsdev->isdb != ISDB_S) {
			ret = -EINVAL;
			break;
		}

		if (!avail) {
			ret = -EIO;
			break;
		}

		if (lnb == 0) {
			// 0V
			b = false;
		} else if (lnb == 2) {
			// 15V
			b = true;
		} else {
			ret = -EINVAL;
			break;
		}

		ret = px4_tsdev_set_lnb_power(tsdev, b);
		break;
	}

	case PTX_DISABLE_LNB_POWER:
		dev_dbg(px4->dev, "px4_tsdev_ioctl %d:%u: PTX_DISABLE_LNB_POWER\n", dev_idx, tsdev_id);

		if (tsdev->isdb != ISDB_S) {
			ret = -EINVAL;
			break;
		}

		if (!avail) {
			ret = -EIO;
			break;
		}

		ret = px4_tsdev_set_lnb_power(tsdev, false);
		break;

	default:
		dev_dbg(px4->dev, "px4_tsdev_ioctl %d:%u: unknown ioctl 0x%08lx\n", dev_idx, tsdev_id, cmd);
		ret = -ENOSYS;
		break;
	}

	mutex_unlock(&tsdev->lock);

	return ret;
}
#endif

static int px4_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	
	if( uaa->usb_mode != USB_MODE_HOST ){
		return ENXIO;
	}
	
	if( (uaa->info.idVendor == 0x0511 ) &&
		( (uaa->info.idProduct == PID_PX_W3U4  ) ||
		  (uaa->info.idProduct == PID_PX_W3PE4 ) ||
		  (uaa->info.idProduct == PID_PX_Q3U4  ) ||
		  (uaa->info.idProduct == PID_PX_Q3PE4 ) ) &&
		(uaa->info.bInterfaceClass == UICLASS_VENDOR ) &&
		(uaa->info.bInterfaceSubClass == 0 ) &&
		(uaa->info.bInterfaceProtocol == 0 )){
		
		pr_info( DEVICE_NAME
#ifdef PX4_DRIVER_VERSION
				 " version " PX4_DRIVER_VERSION
#endif
#ifdef REVISION_NUMBER
#if defined(PX4_DRIVER_VERSION)
				 ","
#endif
				 " rev: " REVISION_NUMBER
#endif
#ifdef COMMIT_HASH
#if defined(PX4_DRIVER_VERSION) || defined(REVISION_NUMBER)
				 ","
#endif
				 " commit: " COMMIT_HASH
#endif
#ifdef REVISION_NAME
				 " @ " REVISION_NAME
#endif
				 "\n");
		
		return BUS_PROBE_SPECIFIC;
	}
	
	return ENXIO;
}

static int px4_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct px4_softc *px4 = device_get_softc(dev);
	struct usb_interface_descriptor *idesc;
	struct usb_config_descriptor *cdesc;
	uint8_t iface_index = uaa->info.bIfaceIndex;
	int error;
	int ret = 0;
	int dev_idx;
	int i, multi_devs_idx = -1;
	int last_power_count = 0;
	const char *serial_str;
	
	struct it930x_bridge *it930x;
	struct it930x_bus *bus;
	
	dev_dbg(dev, "px4_attach: xfer_packets: %u\n", xfer_packets);
	
#if defined(__FreeBSD__)
	sx_xlock(&glock);
#else
	mutex_lock(&glock);
#endif
	
	dev_idx = device_get_unit(dev);
	
	dev_dbg(dev, "px4_attach: dev_idx: %d\n", dev_idx);
	
	if ( devs[dev_idx] != 0) {
		dev_err(dev, "Unused device index was not found.\n");
		ret = -ECANCELED;
		goto fail_before_base;
	}
	
	if( usbd_get_speed( uaa->device ) < USB_SPEED_HIGH )
		dev_warn(dev, "This device is operating as USB 1.1 or less.\n");
	
	
	//px4 = kzalloc(sizeof(*px4), GFP_KERNEL);
	//if (!px4) {
	//  dev_err(dev, "px4_probe: kzalloc(sizeof(*px4), GFP_KERNEL) failed.\n");
	// ret = -ENOMEM;
	//  goto fail_before_base;
	//}
	
	px4->stream_context = (struct px4_stream_context *)kzalloc(sizeof(*px4->stream_context), GFP_ATOMIC);
	if (!px4->stream_context) {
		dev_err(dev, "px4_probe: kzalloc(sizeof(*px4->stream_context), GFP_ATOMIC) failed.\n");
		ret = -ENOMEM;
		goto fail_before_base;
	}

	px4->dev = dev;
	px4->usbdev = uaa->device;
	device_set_usb_desc(dev);
	px4->dev_idx = dev_idx;
	px4->serial_number = 0;
	px4->dev_id = 0;
	px4->multi_dev = NULL;
	
	serial_str =usb_get_serial(uaa->device);
	
	if (strlen(serial_str) == 15) {
		px4->serial_number = strtouq(serial_str, NULL, 16 );
		
		if ( px4->serial_number == ULLONG_MAX)
			dev_err(dev, "px4_probe: strict_strtoull() failed.\n");
		else {
			px4->dev_id = px4->serial_number % 16;

			dev_dbg(dev, "px4_probe: serial_number: %014llx\n", px4->serial_number);
			dev_dbg(dev, "px4_probe: dev_id: %u\n", px4->dev_id);

			if (px4->dev_id != 1 && px4->dev_id != 2)
				dev_warn(dev, "px4_probe: Unexpected device id: %u\n", px4->dev_id);
		}
	} else
		dev_warn(dev, "px4_probe: Invalid serial number length.\n");

	if (uaa->info.idVendor == 0x0511 &&
		(uaa->info.idProduct == PID_PX_Q3U4 || uaa->info.idProduct == PID_PX_Q3PE4)) {
		if (disable_multi_device_power_control)
			dev_info(dev, "Multi device power control: disabled\n");
		else {
			int multi_dev_idx = -1;

			dev_info(dev, "Multi device power control: enabled\n");

			for (i = 0; i < MAX_DEVICE; i++) {
				if (devs[i] != NULL && devs[i]->serial_number == px4->serial_number) {
					multi_dev_idx = i;
					break;
				}
			}

			if (multi_dev_idx != -1) {
				px4->multi_dev = devs[multi_dev_idx]->multi_dev;
			} else {
				px4->multi_dev = kzalloc(sizeof(*px4->multi_dev), GFP_KERNEL);
				if (!px4->multi_dev) {
					dev_err(dev, "px4_probe: kzalloc(sizeof(*px4->multi_dev), GFP_KERNEL) failed. ");
					ret = -ENOMEM;
					goto fail_before_base;
				}

				mutex_init(&px4->multi_dev->lock);
			}

			if (px4->multi_dev) {
				struct px4_multi_device *multi_dev = px4->multi_dev;

				mutex_lock(&multi_dev->lock);

				if (multi_dev->ref > 2)
					px4->multi_dev = NULL;
				else
					multi_devs_idx = multi_dev->ref++;

				mutex_unlock(&multi_dev->lock);
			}
		}
	}
	
	idesc = usbd_get_interface_descriptor( uaa->iface );
	
	for(;;) {
		if (idesc == NULL )
			break;
		
		if ((idesc->bDescriptorType == UDESC_INTERFACE)&&
			(idesc->bLength >= sizeof(*idesc))){
			if (idesc->bInterfaceNumber != uaa->info.bIfaceNum)
				break;
			else {
				goto found;
				
			}
		}
		cdesc = usbd_get_config_descriptor(uaa->device);
		idesc = (void *)usb_desc_foreach(cdesc, (void *)idesc);
		
	}
	goto fail_before_base;
	
found:
	px4->sc_iface_num = idesc->bInterfaceNumber;

	// Initialize px4 structure

	ret = px4_init(px4);
	if (ret)
		goto fail_before_base;

	px4_sysctl_init(px4);

	it930x = &px4->it930x;
	bus = &it930x->bus;
	
	// Initialize bus operator
	
	bus->dev = dev;
	bus->type = IT930X_BUS_USB;
	bus->usb.dev = uaa->device;
	bus->usb.ctrl_timeout = 3000;
	bus->usb.iface_num = idesc->bInterfaceNumber;
	bus->usb.iface_index = iface_index;
	bus->usb.stream_usb_frames = usb_max_frames;
	bus->usb.stream_usb_bufsize = STREAM_BUF_SIZE;

	ret = it930x_bus_init(bus);
	if (ret)
		goto fail_before_bus;

	// Initialize bridge operator

	it930x->dev = dev;
	it930x->config.xfer_size = 188 * xfer_packets;
	it930x->config.i2c_speed = 0x07;

	ret = it930x_init(it930x);
	if (ret)
		goto fail_before_bridge;

	// Load config from eeprom

	ret = px4_load_config(px4);
	if (ret)
		goto fail;

	// Initialize IT930x bridge

	ret = it930x_load_firmware(it930x, FIRMWARE_FILENAME);
	if (ret)
		goto fail;

	ret = it930x_init_device(it930x);
	if (ret)
		goto fail;

	// GPIO configurations

	ret = it930x_set_gpio_mode(it930x, 7, IT930X_GPIO_OUT, true);
	if (ret)
		goto fail;

	ret = it930x_set_gpio_mode(it930x, 2, IT930X_GPIO_OUT, true);
	if (ret)
		goto fail;

	if (px4->multi_dev && multi_devs_idx != -1) {
		mutex_lock(&px4->multi_dev->lock);

		last_power_count = px4->multi_dev->power_count;

		if (px4->multi_dev->power_count) {
			ret = it930x_write_gpio(it930x, 7, false);
			if (ret)
				goto fail;

			msleep(75);

			ret = it930x_write_gpio(it930x, 2, true);
			if (ret)
				goto fail;

			msleep(20);

			px4->multi_dev->power_count++;
		} else {
			ret = it930x_write_gpio(it930x, 7, true);
			if (ret)
				goto fail;

			ret = it930x_write_gpio(it930x, 2, false);
			if (ret)
				goto fail;
		}

		px4->multi_dev->devs[multi_devs_idx] = px4;

		mutex_unlock(&px4->multi_dev->lock);
	} else {
		ret = it930x_write_gpio(it930x, 7, true);
		if (ret)
			goto fail;

		ret = it930x_write_gpio(it930x, 2, false);
		if (ret)
			goto fail;
	}

	ret = it930x_set_gpio_mode(it930x, 11, IT930X_GPIO_OUT, true);
	if (ret)
		goto fail;

	// LNB power supply: off
	ret = it930x_write_gpio(it930x, 11, false);
	if (ret)
		goto fail;

	// create /dev/px4video*
	for( i = 0; i < TSDEV_NUM; i++ ){
		px4_fifo_methods.postfix[0]= i/2 ? "t": "s";

		error= usb_fifo_attach( uaa->device, &px4->tsdev[ i ],
								&px4->tsdev[ i ].fifo_lock,
								&px4_fifo_methods, &px4->sc_fifo[ i ], dev_idx, i,
								iface_index, UID_ROOT, GID_OPERATOR, 0666);
    
		if (error )
			goto fail_fifo;
	}
	dev_dbg(dev, "px4_attach: created /dev/px4video\n");
	devs[dev_idx] = px4;
	
#if defined(__FreeBSD__)
	sx_xunlock(&glock);
#else
	mutex_unlock(&glock);
#endif
	
	return 0;

fail_fifo:
	for (i = 0; i < TSDEV_NUM; i++)
		usb_fifo_detach( &px4->sc_fifo[i]);

fail:
	it930x_term(it930x);
fail_before_bridge:
	it930x_bus_term(bus);
fail_before_bus:
	px4_term(px4);
fail_before_base:
	if (px4) {
		if (px4->multi_dev) {
			struct px4_multi_device *multi_dev = px4->multi_dev;

			mutex_lock(&multi_dev->lock);

			if (multi_dev->power_count > last_power_count)
				multi_dev->power_count--;

			for (i = 0; i < 2; i++) {
				if (multi_dev->devs[i] == px4)
					multi_dev->devs[i] = NULL;
			}

			if (!--multi_dev->ref)
				kfree(multi_dev);
			else
				mutex_unlock(&multi_dev->lock);
		}

		if (px4->stream_context)
			kfree(px4->stream_context);
	}
	
#if defined(__FreeBSD__)
	sx_xunlock(&glock);
#else
	mutex_unlock(&glock);
#endif

	return ret;
}

static int px4_detach(device_t dev)
{
	int i, ref;
	struct px4_softc *px4 = device_get_softc(dev);

	if (!px4)
		return -EINVAL;

	dev_dbg(px4->dev, "px4_disconnect: dev_idx: %d\n", px4->dev_idx);

	atomic_set(&px4->avail, 0);
#if defined(__FreeBSD__)
	sx_xlock( &px4->xlock );
#else
	mutex_lock(&px4->lock);
#endif

#if defined(__FreeBSD__)
	sx_xlock(&glock);
#else
	mutex_lock(&glock);
#endif

	devs[px4->dev_idx] = NULL;

	// delete /dev/px4video*
	for (i = 0; i < TSDEV_NUM; i++)
		usb_fifo_detach( &px4->sc_fifo[i]);

#if defined(__FreeBSD__)
	sx_xunlock(&glock);
#else
	mutex_unlock(&glock);
#endif

	ref = px4_unref(px4);

#if defined(__FreeBSD__)
	sx_xunlock( &px4->xlock );
#else
	mutex_unlock(&px4->lock);
#endif

	for (i = 0; i < TSDEV_NUM; i++) {
		struct px4_tsdev *tsdev = &px4->tsdev[i];

#if defined(__FreeBSD__)
		sx_xlock(&tsdev->xlock);
#else
		mutex_lock(&tsdev->lock);
#endif
		px4_tsdev_stop_streaming(tsdev, false);
#if defined(__FreeBSD__)
		sx_xunlock(&tsdev->xlock);
#else
		mutex_unlock(&tsdev->lock);
#endif
	}

	mutex_lock( &px4->wait_mtx );	
	while (ref) {
		cv_wait( &px4->wait_cv, &px4->wait_mtx);
		ref = atomic_read(&px4->ref);
	}
	mutex_unlock( &px4->wait_mtx );

#if defined(__FreeBSD__)
	sx_xlock(&glock);
#else
	mutex_lock(&glock);
#endif

	if (px4->multi_dev) {
		struct px4_multi_device *multi_dev = px4->multi_dev;

		mutex_lock(&multi_dev->lock);

		for (i = 0; i < 2; i++) {
			if (multi_dev->devs[i] == px4)
				multi_dev->devs[i] = NULL;
		}

		if (!--multi_dev->ref)
			kfree(multi_dev);
		else
			mutex_unlock(&multi_dev->lock);
	}

#if defined(__FreeBSD__)
	sx_xunlock(&glock);
#else
	mutex_unlock(&glock);
#endif

	// uninitialize
	it930x_term(&px4->it930x);
	it930x_bus_term(&px4->it930x.bus);
	px4_term(px4);
	kfree(px4->stream_context);

	return 0;
}
static device_method_t px4_methods[] = {
	/* Device interface. */
	DEVMETHOD(device_probe, px4_probe),
	DEVMETHOD(device_attach, px4_attach),
	DEVMETHOD(device_detach, px4_detach),
	{ 0, 0 }
};

static driver_t px4_driver = {
	DEVICE_NAME,
	px4_methods,
	sizeof(struct px4_softc)
};

static devclass_t px4_devclass;

DRIVER_MODULE(px4, uhub, px4_driver, px4_devclass, 0, 0);
MODULE_DEPEND(px4, usb, 1, 1, 1);


