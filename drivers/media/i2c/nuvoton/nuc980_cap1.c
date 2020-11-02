/*
 * Copyright (c) 2018 Nuvoton Technology Corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf-dma-contig.h>
#include <media/v4l2-dev.h>

#include <media/v4l2-device.h>
#include <linux/jiffies.h>

#include <asm/io.h>

#include <mach/regs-gcr.h>
#include <mach/regs-clock.h>
//#include <mach/regs-lcd.h>
#include <mach/regs-gpio.h>

#include <mach/regs-cap.h>
#include <mach/gpio.h>
#include <linux/time.h>

#include "nuc980_cap.h"
#define CAP1_PD_PIN NUC980_PC0
#define CAP1_RST_PIN NUC980_PE10

u32 sensor1_model = 1;
u32 video1_freq = 24000000;

void nuvoton_vdi1_enable(struct nuvoton_vin_device* cam)
{
	u8 packet=0,planar=0,engine=0;
	ENTRY();
	if(cam->vpe.PacketEnable==1) {
		packet=1;
		if(cam->sensor.bothenable==1)
			planar=1;
	}

	if(cam->vpe.PlanarEnable==1) planar=1;
	engine=(packet|planar);
	__raw_writel( __raw_readl(REG_CAP1_CTL) | ( (engine<<0) | (packet<<6) | (planar<<5) ),REG_CAP1_CTL);
	LEAVE();
}

void nuvoton_vdi1_disable(struct nuvoton_vin_device* cam)
{
	ENTRY();
	if((__raw_readl(REG_CAP1_CTL) & ( (1<<6) | (1<<5) ))!= 0) {
		__raw_writel(__raw_readl(REG_CAP1_CTL)|(1<<16),REG_CAP1_CTL);
		while((__raw_readl(REG_CAP1_CTL) & 1)==1);
		__raw_writel( __raw_readl(REG_CAP1_CTL) & ~(  (1<<6) | (1<<5) ),REG_CAP1_CTL);
	}
	LEAVE();
}

/* ---- IOCTL vidioc handling  ----*/
static int nuvoton_vidioc1_querycap(struct file* file,void* priv,struct v4l2_capability *cap)
{
	struct nuvoton_vin_device* cam=priv;
	ENTRY();
	strlcpy(cap->driver,"nuvoton_vin1",sizeof(cap->driver));
	cap->version = KERNEL_VERSION(1, 1, 10);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING| V4L2_CAP_VIDEO_OVERLAY | V4L2_CAP_DEVICE_CAPS;
	cap->device_caps = V4L2_CAP_EXT_PIX_FORMAT;
	strlcpy(cap->card, cam->v4ldev->name, sizeof(cap->card));
	LEAVE();
	return 0;
}

#if 1
/* set the parameters of frame rate control of capture DMA of NUVOTON */
static int nuvoton_vidioc1_s_parm(struct file* file,void* priv,struct v4l2_streamparm *parm)
{
	ENTRY();
	LEAVE();
	return 0;
}

/* get the frame rate parameters */
static int nuvoton_vidioc1_g_parm(struct file* file,void* priv,struct v4l2_streamparm *parm)
{
	ENTRY();
	LEAVE();
	return 0;
}
#endif

/* enum all supported formats of specific device */
static const struct cap_format cap_formats[] = {
	{
		.desc        = "Packet YUV422",
		.pixelformat = V4L2_PIX_FMT_YUYV,
	},
	{
		.desc        = "Packet GREY",
		.pixelformat = V4L2_PIX_FMT_GREY,
	},
	{
		.desc        = "Packet RGB 565",
		.pixelformat = V4L2_PIX_FMT_RGB565,
	},
	{
		.desc        = "Packet RGB 555",
		.pixelformat = V4L2_PIX_FMT_RGB555,
	},

	{
		.desc        = "Planar YUV422",
		.pixelformat = V4L2_PIX_FMT_YUV422P,
	},
	{
		.desc        = "Planar YUV420",
		.pixelformat = V4L2_PIX_FMT_YUV411P,
	},
};
static int nuvoton_vidioc1_enum_fmt_vid_cap(struct file *file,void  *priv,struct v4l2_fmtdesc *f)
{
	ENTRY();
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (f->index >= sizeof(cap_formats)/sizeof(struct cap_format))
		return -EINVAL;
	strlcpy(f->description,cap_formats[f->index].desc,sizeof(f->description));
	f->pixelformat = cap_formats[f->index].pixelformat;
	LEAVE();
	return 0;
}

/* get the parameters of frame, width, height, field, type, bytesperline, sizeimage */
static int nuvoton_vidioc1_g_fmt_vid_cap(struct file *file,void *priv,struct v4l2_format *format)
{
	struct nuvoton_vin_device* cam=priv;
	struct v4l2_pix_format* pfmt = &(cam->sensor.pix_format);
	ENTRY();
	pfmt->bytesperline = 0;
	pfmt->sizeimage = pfmt->height * ((pfmt->width*pfmt->priv)/8);
	pfmt->field = V4L2_FIELD_NONE;
	memcpy(&(format->fmt.pix), pfmt, sizeof(*pfmt));
	LEAVE();
	return 0;
}

unsigned short GCD1(unsigned short m1, unsigned short m2)
{
	unsigned short m;
	if(m1<m2) {
		m=m1;
		m1=m2;
		m2=m;
	}
	if(m1%m2==0)
		return m2;
	else
		return (GCD1(m2,m1%m2));
}

void nuvoton_cap1_SetPlanarFmt(struct nuvoton_vin_device* cam,struct v4l2_pix_format* pix)
{
	struct nuvoton_vin_sensor* s = &cam->sensor;
	u32 outfmt=0;
	u32 u32GCD;
	if(s->planarfmt==V4L2_PIX_FMT_YUV422P)
		outfmt  |= 0<<7;
	else
		outfmt  |= 1<<7;
	__raw_writel( (__raw_readl(REG_CAP1_PAR) & ~(1<<7)) | outfmt,REG_CAP1_PAR );

	VDEBUG("Planar, pix->height=%d,pix->width=%d\n",pix->height,pix->width);
	VDEBUG("Planar, s->cropcap.bounds.height=%d,s->cropcap.bounds.width=%d\n",s->cropcap.bounds.height,s->cropcap.bounds.width);
	__raw_writel( (__raw_readl(REG_CAP1_PAR) & ~(1<<7)) | outfmt,REG_CAP1_PAR );

	/* Planar Scaling Vertical Factor Register (LSB) */
	u32GCD=pix->height/s->cropcap.bounds.height;
	if(u32GCD<=0) u32GCD=1;
	__raw_writel( (__raw_readl(REG_CAP1_PLNSL) & ~(CAP_PLNSL_PLNSVNL | CAP_PLNSL_PLNSVML))|
	              ( ((pix->height/u32GCD)&0xff)<<24 | ((s->cropcap.bounds.height/u32GCD)&0xff)<<16),REG_CAP1_PLNSL);

	/* Planar Scaling Vertical Factor Register (MSB) */
	u32GCD=pix->height/s->cropcap.bounds.height;
	if(u32GCD<=0) u32GCD=1;
	__raw_writel( (__raw_readl(REG_CAP1_PLNSM) & ~(CAP_PLNSM_PLNSVNH | CAP_PLNSM_PLNSVMH))|
	              ( ((pix->height/u32GCD)>>8)<<24 | ((s->cropcap.bounds.height)>>8)/u32GCD<<16),REG_CAP1_PLNSM);

	/* Planar Scaling Horizontal Factor Register (LSB) */
	u32GCD=pix->width/s->cropcap.bounds.width;
	if(u32GCD<=0) u32GCD=1;
	__raw_writel( (__raw_readl(REG_CAP1_PLNSL) & ~(CAP_PLNSL_PLNSHNL | CAP_PLNSL_PLNSHML))|
	              (((pix->width/u32GCD) & 0xff)<<8 | ((s->cropcap.bounds.width/u32GCD) & 0xff)<<0),REG_CAP1_PLNSL);

	/* Planar Scaling Horizontal Factor Register (MSB) */
	u32GCD=pix->width/s->cropcap.bounds.width;
	if(u32GCD<=0) u32GCD=1;
	__raw_writel( (__raw_readl(REG_CAP1_PLNSM) & ~(CAP_PLNSM_PLNSHNH | CAP_PLNSM_PLNSHMH))|
	              (((pix->width/u32GCD) >>8)<<8 | ((s->cropcap.bounds.width/u32GCD)>>8)<<0),REG_CAP1_PLNSM);

	/* Frame Output Pixel Stride Width Register(Planar) */
#if 0
	__raw_writel( (__raw_readl(REG_CAP1_STRIDE)& ~CAP_STRIDE_PLNSTRIDE) |
	              (/*PacketStride*/pix->width<<16),REG_CAP1_STRIDE);
#else
	__raw_writel( (__raw_readl(REG_CAP1_STRIDE)& ~CAP_STRIDE_PLNSTRIDE) |
	              (/*PacketStride*/(pix->bytesperline/2)<<16),REG_CAP1_STRIDE);
#endif

	cam->vpe.PlanarWidth=pix->width;
	cam->vpe.PlanarHeight=pix->height;
}

/* This ioctl is similar to vidioc_s_fmt_vid_cap(). However, this ioctl is used to query the formats
that the device supports without changing any state of the device. */
static int nuvoton_vidioc1_try_fmt_vid_cap(struct file *file, void *priv,struct v4l2_format *format)
{
	struct nuvoton_vin_device* cam=priv;
	struct nuvoton_vin_sensor* s = &cam->sensor;
	struct v4l2_pix_format* pix;
	struct v4l2_pix_format* pfmt = &(s->pix_format);
	struct v4l2_rect* bounds = &(s->cropcap.bounds);
	struct v4l2_rect rect;
	u32 u32GCD1;
	u32 heightM,heightN,widthM,widthN;
	u32 outfmt;
	ENTRY();
	pix = &(format->fmt.pix);

	if (format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	memcpy(&rect, &(s->_rect), sizeof(rect));

	rect.width = pix->width;
	rect.height = pix->height;
	if (rect.width < 8)
		rect.width = 8;
	if (rect.height < 8)
		rect.height = 8;
	if (rect.width > bounds->left + bounds->width - rect.left)
		rect.width = bounds->left + bounds->width - rect.left;
	if (rect.height > bounds->top + bounds->height - rect.top)
		rect.height = bounds->top + bounds->height - rect.top;
	rect.width &= ~7L;
	rect.height &= ~7L;

	pix->width = rect.width;
	pix->height = rect.height;
	pix->priv = pfmt->priv;
	pix->colorspace = pfmt->colorspace;
	if(pix->bytesperline==0) {
		pix->bytesperline = pix->width*2;
		pfmt->bytesperline = pix->width*2;
	}

	pix->sizeimage = pix->height * (((pix->bytesperline/2) * pix->priv) / 8);
	pix->field = V4L2_FIELD_NONE;
	memcpy(pfmt, pix, sizeof(*pix));
	/*Set capture format for nuvoton sensor interface */
	__raw_writel( (__raw_readl(REG_CAP1_CWS) & ~(0x0fff0fff)) | (s->cropcap.bounds.width) | (s->cropcap.bounds.height<<16),REG_CAP1_CWS );
	switch(pix->pixelformat) {
		/* Packet YUV422 */
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_GREY:
		VDEBUG("Packet\n");
		if(pix->pixelformat==V4L2_PIX_FMT_YUYV)
			outfmt = 0<<4;
		if(pix->pixelformat==V4L2_PIX_FMT_GREY) {
			pix->priv = 8;
			outfmt = 1<<4;
		}
		if(pix->pixelformat==V4L2_PIX_FMT_RGB555)
			outfmt = 2<<4;
		if(pix->pixelformat==V4L2_PIX_FMT_RGB565)
			outfmt = 3<<4; //infmtord

		if(s->bothenable==1) {
			nuvoton_cap1_SetPlanarFmt(cam,pix);
		}

		__raw_writel( (__raw_readl(REG_CAP1_PAR) & ~(3<<4)) | outfmt,REG_CAP1_PAR );
		__raw_writel( (__raw_readl(REG_CAP1_PAR) & ~INMASK) | s->infmtord,REG_CAP1_PAR );
		//VDEBUG("pix->pixelformat = V4L2_PIX_FMT_YUYV\n");
		/* Set_Cropping start position for sensor */
		if(cam->users==1)
			__raw_writel( (__raw_readl(REG_CAP1_CWSP) & ~(CAP_CWSP_CWSADDRV | CAP_CWSP_CWSADDRH)) | (s->cropstart),REG_CAP1_CWSP );

		/* Packet Scaling Vertical Factor Register (LSB) */
		VDEBUG("pix->height=%d, s->cropcap.bounds.height = %d\n",pix->height,s->cropcap.bounds.height);
		if(pix->height > s->cropcap.bounds.height) {
			heightN=1;
			heightM=1;
		} else {
			heightM = s->cropcap.bounds.height;
			heightN = pix->height;
		}
		__raw_writel( (__raw_readl(REG_CAP1_PKTSL) & ~(CAP_PKTSL_PKTSVNL | CAP_PKTSL_PKTSVML))|
		              ((heightN & 0xff)<<24|(heightM & 0xff)<<16),REG_CAP1_PKTSL);

		/* Packet Scaling Vertical Factor Register (MSB) */
		__raw_writel( (__raw_readl(REG_CAP1_PKTSM) & ~(CAP_PKTSM_PKTSVNH | CAP_PKTSM_PKTSVMH))|
		              ((heightN>>8)<<24|(heightM>>8)<<16),REG_CAP1_PKTSM);
		/* Packet Scaling Horizontal Factor Register (LSB) */
		VDEBUG("pix->width=%d, s->cropcap.bounds.width = %d\n",pix->width,s->cropcap.bounds.width);
		if(pix->width > s->cropcap.bounds.width) {
			widthN=1;
			widthM=1;
		} else {
			widthM = s->cropcap.bounds.width;
			widthN = pix->width;
		}
		__raw_writel( (__raw_readl(REG_CAP1_PKTSL) & ~(CAP_PKTSL_PKTSHNL | CAP_PKTSL_PKTSHML))|
		              ((widthN & 0xff)<<8 | (widthM & 0xff)<<0),REG_CAP1_PKTSL);

		/* Packet Scaling Horizontal Factor Register (MSB) */
		__raw_writel( (__raw_readl(REG_CAP1_PKTSM) & ~(CAP_PKTSM_PKTSHNH | CAP_PKTSM_PKTSHMH))|
		              ((widthN>>8)<<8 | (widthM>>8)<<0),REG_CAP1_PKTSM);

		/* Frame Output Pixel Stride Width Register(Packet/Planar) */
#if 0
		__raw_writel( (__raw_readl(REG_CAP1_STRIDE)& ~CAP_STRIDE_PKTSTRIDE) |
		              (/*PacketStride*/pix->width<<0),REG_CAP1_STRIDE);
#else
		if(pix->pixelformat!=V4L2_PIX_FMT_GREY) {
			__raw_writel( (__raw_readl(REG_CAP1_STRIDE)& ~CAP_STRIDE_PKTSTRIDE) |
			              (/*PacketStride*/(pix->bytesperline/2)<<0),REG_CAP1_STRIDE);
		} else {
			__raw_writel( (__raw_readl(REG_CAP1_STRIDE)& ~CAP_STRIDE_PKTSTRIDE) |
			              (/*PacketStride*/(pix->bytesperline/1)<<0),REG_CAP1_STRIDE);
		}
#endif
		cam->vpe.format=pix->pixelformat;
		cam->vpe.PacketWidth=pix->width;
		cam->vpe.PacketHeight=pix->height;
		cam->vpe.PacketEnable=1;
		break;

		/* Planar YUV422 */
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUV411P:
		if(pix->pixelformat==V4L2_PIX_FMT_YUV422P)  outfmt = 0<<7;
		if(pix->pixelformat==V4L2_PIX_FMT_YUV411P)  outfmt = 1<<7;
		VDEBUG("Planar, cam->users=%d\n",cam->users);
		VDEBUG("Planar, pix->height=%d,pix->width=%d\n",pix->height,pix->width);
		VDEBUG("Planar, s->cropcap.bounds.height=%d,s->cropcap.bounds.width=%d\n",s->cropcap.bounds.height,s->cropcap.bounds.width);
		__raw_writel( (__raw_readl(REG_CAP1_PAR) & ~(1<<7)) | outfmt,REG_CAP1_PAR );
		//VDEBUG("pix->pixelformat = V4L2_PIX_FMT_YUV422P\n");
		/* Set_Cropping start position for sensor */
		if(cam->users==1)
			__raw_writel( (__raw_readl(REG_CAP1_CWSP) & ~(CAP_CWSP_CWSADDRV | CAP_CWSP_CWSADDRH)) | ( 0 | 2<<16 ),REG_CAP1_CWSP );
		/* Planar Scaling Vertical Factor Register (LSB) */
		u32GCD1=pix->height/s->cropcap.bounds.height;
		if(u32GCD1<=0) u32GCD1=1;
		__raw_writel( (__raw_readl(REG_CAP1_PLNSL) & ~(CAP_PLNSL_PLNSVNL | CAP_PLNSL_PLNSVML))|
		              ( ((pix->height/u32GCD1)&0xff)<<24 | ((s->cropcap.bounds.height/u32GCD1)&0xff)<<16),REG_CAP1_PLNSL);

		/* Planar Scaling Vertical Factor Register (MSB) */
		u32GCD1=pix->height/s->cropcap.bounds.height;
		if(u32GCD1<=0) u32GCD1=1;
		__raw_writel( (__raw_readl(REG_CAP1_PLNSM) & ~(CAP_PLNSM_PLNSVNH | CAP_PLNSM_PLNSVMH))|
		              ( ((pix->height/u32GCD1)>>8)<<24 | ((s->cropcap.bounds.height)>>8)/u32GCD1<<16),REG_CAP1_PLNSM);

		/* Planar Scaling Horizontal Factor Register (LSB) */
		u32GCD1=pix->width/s->cropcap.bounds.width;
		if(u32GCD1<=0) u32GCD1=1;
		__raw_writel( (__raw_readl(REG_CAP1_PLNSL) & ~(CAP_PLNSL_PLNSHNL | CAP_PLNSL_PLNSHML))|
		              (((pix->width/u32GCD1) & 0xff)<<8 | ((s->cropcap.bounds.width/u32GCD1) & 0xff)<<0),REG_CAP1_PLNSL);

		/* Planar Scaling Horizontal Factor Register (MSB) */
		u32GCD1=pix->width/s->cropcap.bounds.width;
		if(u32GCD1<=0) u32GCD1=1;
		__raw_writel( (__raw_readl(REG_CAP1_PLNSM) & ~(CAP_PLNSM_PLNSHNH | CAP_PLNSM_PLNSHMH))|
		              (((pix->width/u32GCD1) >>8)<<8 | ((s->cropcap.bounds.width/u32GCD1)>>8)<<0),REG_CAP1_PLNSM);

		/* Frame Output Pixel Stride Width Register(Planar) */
#if 0
		__raw_writel( (__raw_readl(REG_CAP1_STRIDE)& ~CAP_STRIDE_PLNSTRIDE) |
		              (/*PacketStride*/pix->width<<16),REG_CAP1_STRIDE);
#else
		__raw_writel( (__raw_readl(REG_CAP1_STRIDE)& ~CAP_STRIDE_PLNSTRIDE) |
		              (/*PacketStride*/(pix->bytesperline/2)<<16),REG_CAP1_STRIDE);
#endif

		cam->vpe.format=pix->pixelformat;
		cam->vpe.PlanarWidth=pix->width;
		cam->vpe.PlanarHeight=pix->height;
		cam->vpe.PlanarEnable=1;
		VDEBUG("V4L2_PIX_FMT_YUV422P END\n",cam->users);
		break;

#if 0  /* not surpport yet, TODO: */
	case V4L2_PIX_FMT_NV16: /* 16  Y/CbCr 4:2:2  */
		break;
	case V4L2_PIX_FMT_NV61: /* 16  Y/CrCb 4:2:2  */
		break;
	case V4L2_PIX_FMT_YYUV:
		break;
	case V4L2_PIX_FMT_YUYV:
		break;
	case V4L2_PIX_FMT_YYUV:
		break;
	case V4L2_PIX_FMT_YVYU:
		break;
	case V4L2_PIX_FMT_UYVY:
		break;
	case V4L2_PIX_FMT_VYUY:
		break;

		/* packet only Y*/
	case V4L2_PIX_FMT_GREY:
		break;

		/* Packet RGB555 */
	case V4L2_PIX_FMT_RGB555:
		break;

		/* Packet RGB565 */
	case V4L2_PIX_FMT_RGB565:
		break;

#endif

	default :
		return -EINVAL;
		break;

	}

	if(cam->users==1) {
		__raw_writel((__raw_readl(REG_CAP1_PAR) & ~(VSP_HI|HSP_HI|PCLKP_HI)) | s->polarity,REG_CAP1_PAR); /* set CAP Polarity */
		__raw_writel((__raw_readl(REG_CAP1_INT) | 0x10000) ,REG_CAP1_INT);     /* Enable CAP Interrupt */
	}

	LEAVE();
	return 0;
}


/* FIX ME: This seems to be generic enough to be at videodev2 */
/* FIX ME:
This function sets the output image resolution of the sensor, and save the parameters. */
static int nuvoton_vidioc1_s_fmt_vid_cap(struct file *file,void *priv,struct v4l2_format *format)
{
	ENTRY();
	nuvoton_vidioc1_try_fmt_vid_cap(file,priv,format);
	LEAVE();
	return 0;
}

/* ask kernel to allocate needed buffer */
static int nuvoton_vidioc1_reqbufs(struct file *file, void *priv,struct v4l2_requestbuffers *rb)
{
	struct nuvoton_vin_device* cam=priv;
	//struct nuvoton_vin_sensor* s = &cam->sensor;
	u32 i;
	int err;
	ENTRY();
	if (rb->type != V4L2_BUF_TYPE_VIDEO_CAPTURE || rb->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;

	if (cam->io == IO_READ) {
		VDEBUG("Close and open the device again to choose the mmap I/O method\n");
		return -EBUSY;
	}

	for (i = 0; i < cam->nbuffers; i++)
		if (cam->frame[i].vma_use_count) {
			VDEBUG("VIDIOC_REQBUFS failed. Previous buffers are still mapped.\n");
			return -EBUSY;
		}

	if (cam->stream == STREAM_ON)
		if ((err = nuvoton_vin1_stream_interrupt(cam)))
			return err;

	nuvoton_vin1_empty_framequeues(cam);

	nuvoton_vin1_release_buffers(cam);
	if (rb->count)
		rb->count = nuvoton_vin1_request_buffers(cam, rb->count, IO_MMAP);
	cam->io = rb->count ? IO_MMAP : IO_NONE;

	LEAVE();
	return 0;
}

/* query buffer info */
static int nuvoton_vidioc1_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct nuvoton_vin_device* cam=priv;
	ENTRY();
	if (p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    p->index >= cam->nbuffers || cam->io != IO_MMAP) {
		LEAVE();
		return -EINVAL;
	}

	memcpy(p,&(cam->frame[p->index].buf),sizeof(struct v4l2_buffer));

	if (cam->frame[p->index].vma_use_count)
		p->flags |= V4L2_BUF_FLAG_MAPPED;

	if (cam->frame[p->index].state == F_DONE)
		p->flags |= V4L2_BUF_FLAG_DONE;
	else if (cam->frame[p->index].state != F_UNUSED)
		p->flags |= V4L2_BUF_FLAG_QUEUED;

	LEAVE();
	return 0;
}

/* put the buffer into the list */
static int nuvoton_vidioc1_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct nuvoton_vin_device* cam=priv;
	struct v4l2_buffer b;
	unsigned long lock_flags;
	ENTRY();
	memcpy(&b,p,sizeof(b));

	if (b.type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    b.index >= cam->nbuffers || cam->io != IO_MMAP) {
		return -EINVAL;
	}
	if (cam->frame[b.index].state != F_UNUSED) {
		return -EINVAL;
	}

	spin_lock_irqsave(&cam->queue_lock, lock_flags);
	cam->frame[b.index].state = F_QUEUED;
	list_add_tail(&cam->frame[b.index].frame, &cam->inqueue);
	spin_unlock_irqrestore(&cam->queue_lock, lock_flags);

	VDEBUG("Frame #%lu queued", (unsigned long)b.index);
	LEAVE();
	return 0;
}

/* de-queue a buffer from the list */
static int nuvoton_vidioc1_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct nuvoton_vin_device* cam=priv;
	struct v4l2_buffer b;
	struct nuvoton_vin_frame_t *f;
	unsigned long lock_flags;
	int err = 0;
	ENTRY();
	memcpy(&b,p,sizeof(b));
	if (b.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (list_empty(&cam->outqueue)) {
		if (cam->stream == STREAM_OFF)
			return -EINVAL;
		err = wait_event_interruptible( cam->wait_frame,(!list_empty(&cam->outqueue)));
		if (err) return err;
	}
	spin_lock_irqsave(&cam->queue_lock, lock_flags);
	f = list_entry(cam->outqueue.next, struct nuvoton_vin_frame_t, frame);
	list_del(cam->outqueue.next);
	spin_unlock_irqrestore(&cam->queue_lock, lock_flags);
	f->state = F_UNUSED;
	b = f->buf;
	if (f->vma_use_count)
		b.flags |= V4L2_BUF_FLAG_MAPPED;
	memcpy(p,&b,sizeof(b));
	VDEBUG("Frame1 #%lu dequeued", (unsigned long)f->buf.index);

	LEAVE();
	return 0;
}

/* start capturing the sensor output stream */
static int nuvoton_vidioc1_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct nuvoton_vin_device* cam=priv;
	ENTRY();

	/* Setting Buffer address */
	VDEBUG("cam->nbuffers=%d\n",cam->nbuffers);

	if(cam->nbuffers>0) {
		VDEBUG("cam->frame_current->bufmem=0x%08x\n",cam->frame_current->bufmem);
		VDEBUG("cam->frame_current->pbuf=0x%08x\n",cam->frame_current->pbuf);
		VDEBUG("cam->vpe.PacketEnable=%d\n",cam->vpe.PacketEnable);

		if(cam->vpe.PacketEnable==1) {
			if(cam->frame_current!=NULL)
				__raw_writel(cam->frame_current->pbuf,REG_CAP1_PKTBA0);
			else
				VDEBUG("PacketEnable : cam->frame_current == NULL\n");

			if(cam->sensor.bothenable==1) {
				struct nuvoton_vin_sensor* s = &cam->sensor;
				struct v4l2_rect* bounds = &(s->cropcap.bounds);
				u32 size = bounds->height*bounds->height*2;
				__raw_writel((unsigned int)cam->frame_current->pbuf+size,REG_CAP1_YBA);
				__raw_writel(__raw_readl(REG_CAP1_YBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight),REG_CAP1_UBA);
				__raw_writel(__raw_readl(REG_CAP1_UBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight)/2,REG_CAP1_VBA);
			}
		}

		VDEBUG("cam->vpe.PlanarEnable=%d\n",cam->vpe.PlanarEnable);
		if(cam->vpe.PlanarEnable==1) {
			if(cam->frame_current!=NULL) {
				__raw_writel((unsigned int)cam->frame_current->pbuf,REG_CAP1_YBA);
				__raw_writel(__raw_readl(REG_CAP1_YBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight),REG_CAP1_UBA);
				__raw_writel(__raw_readl(REG_CAP1_UBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight)/2,REG_CAP1_VBA);
			} else
				VDEBUG("PlanarEnable : cam->frame_current == NULL\n");
		}
	}
	__raw_writel((__raw_readl(REG_CAP1_INT) | 0x10000) ,REG_CAP1_INT);     /* Enable CAP Interrupt */
	/* Capture engine enable and packer/planar mode enable */
	nuvoton_vdi1_enable(cam);

	cam->stream = STREAM_ON;
	LEAVE();
	return 0;
}

/* stop capturing sensor output stream */
static int nuvoton_vidioc1_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct nuvoton_vin_device* cam=priv;
	ENTRY();
	cam->vpe.PacketEnable=0;
	cam->vpe.PacketEnable=0;
	cam->stream = STREAM_OFF;
	nuvoton_vdi1_disable(cam);
	LEAVE();
	return 0;
}

/* setup the device support standard */
static int nuvoton_vidioc1_s_std(struct file *file, void *priv, v4l2_std_id i)
{
	ENTRY();
	LEAVE();
	return 0;
}


/* only one input in this sample driver */
/* Enumerate the input. AVID Daytona supports several input devices
* 1. image sensor
* 2. native image post-processor
* 3. analog TV tuner * * */
static int nuvoton_vidioc1_enum_input(struct file *file,void *priv,struct v4l2_input *inp)
{
	ENTRY();
	strcpy(inp->name, "Camera");
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	LEAVE();
	return 0;
}

/* always returns 0 (we have only one input) */
static int nuvoton_vidioc1_g_input(struct file *file, void *priv, unsigned int *i)
{
	int index = 0;
	ENTRY();
	*i=index;
	LEAVE();
	return 0;
}

/* Because we have only one input, set input = 0 */
static int nuvoton_vidioc1_s_input(struct file *file, void *priv, unsigned int i)
{
	ENTRY();
	if (i != 0) {
		LEAVE();
		return -EINVAL;
	}
	LEAVE();
	return 0;
}

/* ----- *           controls * ----- */
/* Check if the control method is supported (via  v4l2_queryctrl)
If yes, the parameter of the control will be returned;
if not, error code will be returned. */
static int nuvoton_vidioc1_queryctrl(struct file *file,void *priv,struct v4l2_queryctrl *qc)
{

	struct nuvoton_vin_device* cam=priv;
	struct nuvoton_vin_sensor* s = &cam->sensor;
	ENTRY();
	if (qc->id && qc->id == s->qctrl.id) {
		memcpy((char *)qc, (char *)&(s->qctrl), sizeof(qc));
		LEAVE();
		return 0;
	}
	LEAVE();
	return -EINVAL;
}

/* get the control parameter */
static int nuvoton_vidioc1_g_ctrl(struct file *file, void *priv,struct v4l2_control *ctrl)
{
	struct nuvoton_vin_device* cam=priv;
	struct nuvoton_vin_sensor* s = &cam->sensor;
	int err = 0;
	ENTRY();
	if (!s->get_ctrl && !s->set_ctrl)
		return -EINVAL;
	if (!s->get_ctrl) {
		if (ctrl->id == s->qctrl.id) {
			ctrl->value = s->_qctrl.default_value;
			goto exit;
		}
	} else
		err = s->get_ctrl(cam, ctrl);

exit:
	LEAVE();
	return err;
}

/* set the control parameter */
static int nuvoton_vidioc1_s_ctrl(struct file *file,void *priv,struct v4l2_control *ctrl)
{
	struct nuvoton_vin_device* cam=priv;
	struct nuvoton_vin_sensor* s = &cam->sensor;
	int err = 0;
	ENTRY();
	if (!s->set_ctrl)
		return -EINVAL;

	if (ctrl->id == s->qctrl.id) {
		if (s->qctrl.flags & V4L2_CTRL_FLAG_DISABLED)
			return -EINVAL;
		if (ctrl->value < s->qctrl.minimum || ctrl->value > s->qctrl.maximum)
			return -ERANGE;
		ctrl->value -= ctrl->value % s->qctrl.step;
	}
	if ((err = s->set_ctrl(cam, ctrl)))
		return err;
	s->_qctrl.default_value = ctrl->value;
	LEAVE();
	return 0;
}

static int nuvoton_vidioc1_cropcap(struct file *file, void *priv, struct v4l2_cropcap *crop)
{
	struct nuvoton_vin_device* cam=priv;
	struct v4l2_cropcap* cc = &(cam->sensor.cropcap);

	cc->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cc->pixelaspect.numerator = 1;
	cc->pixelaspect.denominator = 1;
	memcpy(crop,cc,sizeof(struct v4l2_cropcap));
	return 0;
}

static int nuvoton_vidioc1_g_crop(struct file *file, void *priv,struct v4l2_crop *crop)
{
	struct nuvoton_vin_device* cam=priv;
	struct nuvoton_vin_sensor* s = &cam->sensor;
	ENTRY();
	crop->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	memcpy(&(crop->c), &(s->_rect), sizeof(struct v4l2_rect));
	LEAVE();
	return 0;
}
static int nuvoton_vidioc1_s_crop(struct file *file, void *priv,const struct v4l2_crop *crop)
{
	struct nuvoton_vin_device* cam=priv;
	struct nuvoton_vin_sensor* s = &cam->sensor;
	struct v4l2_rect* rect;
	struct v4l2_rect* bounds = &(s->cropcap.bounds);
	//const enum unvoton_vin_stream_state stream = cam->stream;
	const u8 stream = cam->stream;
	const u32 nbuffers = cam->nbuffers;
	int err = 0;
	ENTRY();
	rect = (struct v4l2_rect*)&(crop->c);

	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (!s->set_crop) {
		memcpy(rect, &(s->_rect), sizeof(*rect));
		return 0;
	}
	rect->left &= ~7L;
	rect->top &= ~7L;
	if (rect->width < 8)    rect->width = 8;
	if (rect->height < 8)   rect->height = 8;
	if (rect->width > bounds->width)        rect->width = bounds->width;
	if (rect->height > bounds->height)  rect->height = bounds->height;
	if (rect->left < bounds->left)  rect->left = bounds->left;
	if (rect->top < bounds->top)        rect->top = bounds->top;
	if (rect->left + rect->width > bounds->left + bounds->width)    rect->left = bounds->left+bounds->width - rect->width;
	if (rect->top + rect->height > bounds->top + bounds->height)    rect->top = bounds->top+bounds->height - rect->height;
	rect->width &= ~7L;
	rect->height &= ~7L;

	if (cam->stream == STREAM_ON)
		if ((err = nuvoton_vin1_stream_interrupt(cam)))
			return err;


	if (cam->io == IO_READ)
		nuvoton_vin1_release_buffers(cam);

	if (s->set_crop)
		err += s->set_crop(cam, rect);

	if (err) { /* atomic, no rollback in ioctl() */
		//cam->state |= DEV_MISCONFIGURED;
		VDEBUG("VIDIOC_S_CROP failed because of hardware problems. To use the camera, close and open %s again.\n",
		       video_device_node_name(cam->v4ldev));
		return -EIO;
	}

	s->pix_format.width = rect->width;
	s->pix_format.height = rect->height;
	memcpy(&(s->_rect), rect, sizeof(*rect));

	if ((cam->io == IO_READ) && nbuffers != nuvoton_vin1_request_buffers(cam, nbuffers, cam->io)) {
		//cam->state |= DEV_MISCONFIGURED;
		VDEBUG("VIDIOC_S_CROP failed because of not enough memory. To use the camera, close and open %s again.\n",
		       video_device_node_name(cam->v4ldev));
		return -ENOMEM;
	}

	if (cam->io == IO_READ)
		nuvoton_vin1_empty_framequeues(cam);

	cam->stream = stream;
	LEAVE();
	return 0;
}

static int nuvoton_vin1_vidioc_s_jpegcomp(struct file *file, void *priv,const struct v4l2_jpegcompression *jc)
{
	ENTRY();
	LEAVE();
	return 0;
}

static int nuvoton_vidioc1_s_fbuf(struct file *file, void *priv,
                                  const struct v4l2_framebuffer *fb)
{
	int i;
	struct nuvoton_vin_device* cam=priv;
	ENTRY();
	if (fb->flags & V4L2_FBUF_FLAG_OVERLAY) {
		cam->nbuffers = CONFIG_VIN1_MAX_FRAME_BUFFER;
		for (i = 0; i < cam->nbuffers; i++) {
			cam->frame[i].bufmem = cam->vir_addr[i];
			cam->frame[i].pbuf = cam->phy_addr[i];
			cam->frame[i].buf.index = i;
		}
		cam->frame_current=&cam->frame[0];
		__raw_writel(cam->frame_current->pbuf,REG_CAP1_PKTBA0);
		cam->type=V4L2_BUF_TYPE_VIDEO_OVERLAY;
	} else {
		cam->type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
	}
	LEAVE();
	return 0;

}

static int nuvoton_vidioc1_g_fbuf(struct file *file, void *priv,
                                  struct v4l2_framebuffer *fb)
{
//Bttv-driver.c
//Fsl-viu.c
//Ivtv-ioctl.c
//Omap_vout.c
//saa7134-video.c
//Zoran_driver.c
	ENTRY();
	//struct nuvoton_vin_device* cam=priv;
	//struct nuvoton_vin_sensor* s = &cam->sensor;
	fb->capability = V4L2_FBUF_FLAG_OVERLAY;
	fb->flags = V4L2_FBUF_FLAG_OVERLAY;
	LEAVE();
	return 0;
}

/* ---- Initialization v4l2 ioctrl ops   ----*/
static const struct v4l2_ioctl_ops nuvoton_vdi1_ioctl_ops = {
	.vidioc_querycap    = nuvoton_vidioc1_querycap,
	.vidioc_g_fmt_vid_cap   = nuvoton_vidioc1_g_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = nuvoton_vidioc1_enum_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap   = nuvoton_vidioc1_s_fmt_vid_cap,
	.vidioc_enum_input = nuvoton_vidioc1_enum_input,
	.vidioc_g_input = nuvoton_vidioc1_g_input,
	.vidioc_s_input = nuvoton_vidioc1_s_input,
	.vidioc_s_std   = nuvoton_vidioc1_s_std,
	.vidioc_reqbufs = nuvoton_vidioc1_reqbufs,
	.vidioc_try_fmt_vid_cap = nuvoton_vidioc1_try_fmt_vid_cap,
	.vidioc_querybuf    = nuvoton_vidioc1_querybuf,
	.vidioc_qbuf    = nuvoton_vidioc1_qbuf,
	.vidioc_dqbuf   = nuvoton_vidioc1_dqbuf,
	.vidioc_streamon    = nuvoton_vidioc1_streamon,
	.vidioc_streamoff   = nuvoton_vidioc1_streamoff,
	.vidioc_queryctrl   = nuvoton_vidioc1_queryctrl,
	.vidioc_g_ctrl = nuvoton_vidioc1_g_ctrl,
	.vidioc_s_ctrl = nuvoton_vidioc1_s_ctrl,
	.vidioc_cropcap = nuvoton_vidioc1_cropcap,
	.vidioc_g_crop = nuvoton_vidioc1_g_crop,
	.vidioc_s_crop = nuvoton_vidioc1_s_crop,
	.vidioc_s_jpegcomp = nuvoton_vin1_vidioc_s_jpegcomp,
	.vidioc_g_fbuf = nuvoton_vidioc1_g_fbuf,
	.vidioc_s_fbuf = nuvoton_vidioc1_s_fbuf,
	.vidioc_s_parm = nuvoton_vidioc1_s_parm,
	.vidioc_g_parm = nuvoton_vidioc1_g_parm,
	/*  VIDIOC_G_FBUF and VIDIOC_S_FBUF ioctl to get and set
	    the framebuffer parameters for a Video Overlay
	    or Video Output Overlay (OSD) */
};


/*****************************************************************************/
int nuvoton_vin1_stream_interrupt(struct nuvoton_vin_device* cam)
{
	return 0;
}

u32 nuvoton_vin1_request_buffers(struct nuvoton_vin_device* cam, u32 count,enum nuvoton_vin_io_method io)
{
	struct v4l2_pix_format* p = &(cam->sensor.pix_format);
	size_t imagesize;
	//void* buff = NULL;
	u32 i;

	if(p->bytesperline==0)
		imagesize = (p->width * p->height * p->priv) / 8;
	else
		imagesize = ((p->bytesperline/2) * p->height * p->priv) / 8;

	ENTRY();
	if (count > CONFIG_VIN1_MAX_FRAME_BUFFER)
		count = CONFIG_VIN1_MAX_FRAME_BUFFER;

	cam->nbuffers = (count);
	for (i = 0; i < cam->nbuffers; i++) {
		cam->frame[i].bufmem = cam->vir_addr[i];
		cam->frame[i].pbuf = cam->phy_addr[i];
		cam->frame[i].buf.index = i;
		cam->frame[i].buf.m.userptr = (unsigned long)(cam->frame[i].bufmem);
		cam->frame[i].buf.m.offset = (unsigned long)(cam->frame[i].bufmem);
		cam->frame[i].buf.length = imagesize;
		cam->frame[i].buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cam->frame[i].buf.sequence = 0;
		cam->frame[i].buf.field = V4L2_FIELD_NONE;
		cam->frame[i].buf.memory = V4L2_MEMORY_MMAP;
		cam->frame[i].buf.flags = 0;
		VDEBUG("cam->frame[%d].bufmem=0x%08x\n",i,(unsigned int)cam->frame[i].bufmem);
		VDEBUG("cam->frame[%d].pbuf=0x%08x\n",i,(unsigned int)cam->frame[i].pbuf);
	}
	cam->frame_current=&cam->frame[0];
	LEAVE();
	return (cam->nbuffers);
}


void nuvoton_vin1_release_buffers(struct nuvoton_vin_device* cam)
{
	ENTRY();
	if (cam->nbuffers) {
		cam->nbuffers = 0;
	}
	cam->frame_current = NULL;
	LEAVE();
}

void nuvoton_vin1_empty_framequeues(struct nuvoton_vin_device* cam)
{
	u32 i;
	ENTRY();
	INIT_LIST_HEAD(&cam->inqueue);
	INIT_LIST_HEAD(&cam->outqueue);

	for (i = 0; i < CONFIG_VIN1_MAX_FRAME_BUFFER; i++) {
		cam->frame[i].state = F_UNUSED;
		cam->frame[i].buf.bytesused = 0;
	}
	LEAVE();
}

void nuvoton_vin1_requeue_outqueue(struct nuvoton_vin_device* cam)
{
	struct nuvoton_vin_frame_t *i;

	list_for_each_entry(i, &cam->outqueue, frame) {
		i->state = F_QUEUED;
		list_add(&i->frame, &cam->inqueue);
	}
	INIT_LIST_HEAD(&cam->outqueue);
}


static void nuvoton_vin1_queue_unusedframes(struct nuvoton_vin_device* cam)
{
	unsigned long lock_flags;
	u32 i;

	for (i = 0; i < cam->nbuffers; i++)
		if (cam->frame[i].state == F_UNUSED) {
			cam->frame[i].state = F_QUEUED;
			spin_lock_irqsave(&cam->queue_lock, lock_flags);
			list_add_tail(&cam->frame[i].frame, &cam->inqueue);
			spin_unlock_irqrestore(&cam->queue_lock, lock_flags);
		}
}

static void nuvoton_vin1_release_resources(struct kref *kref)
{
	struct nuvoton_vin_device *cam = container_of(kref, struct nuvoton_vin_device,kref);
	ENTRY();
	VDEBUG("V4L2 device %s deregistered\n",video_device_node_name(cam->v4ldev));
	video_set_drvdata(cam->v4ldev, NULL);
	video_unregister_device(cam->v4ldev);
	kfree(cam->control_buffer);
	kfree(cam);
}


int capture1_uninit(void)
{

	__raw_writel( (__raw_readl(REG_MFP_GPD_L) & ~0x000F0000) ,REG_MFP_GPI_L);
	__raw_writel(((__raw_readl(GPIO_BA+0x0C0)&~0x300) | 0x0100),(GPIO_BA+0x0C0)); /* GPIOD4 Output mode */
	__raw_writel((__raw_readl(GPIO_BA+0x0C8) | 0x0010),(GPIO_BA+0x0C8)); /* GPIOD4 Output to High */
	return 0;
}
int capture1_init(struct nuvoton_vin_device* cam)
{
	int ret;
	struct clk *clkcap,*clkaplldiv,*clkmux;
	struct clk *clk;
	int i32Div;
	ENTRY();
	clk = clk_get(NULL, "cap1_eclk");
	if (IS_ERR(clk)) {
		return -ENOENT;
	}
	clk_prepare(clk);
	clk_enable(clk);
	clk_prepare(clk_get(NULL, "cap1_hclk"));
	clk_enable(clk_get(NULL, "cap1_hclk"));
	clk_prepare(clk_get(NULL, "sensor_hclk"));
	clk_enable(clk_get(NULL, "sensor_hclk"));
	clkmux = clk_get(NULL, "cap1_eclk_mux");
	if (IS_ERR(clkmux)) {
		printk(KERN_ERR "nuc980-cap1:failed to get cap clock source\n");
		ret = PTR_ERR(clkmux);
		return ret;
	}
	clkcap = clk_get(NULL, "cap1_eclk");
	if (IS_ERR(clkcap)) {
		printk(KERN_ERR "nuc980-cap1:failed to get cap clock source\n");
		ret = PTR_ERR(clkcap);
		return ret;
	}
	clkaplldiv = clk_get(NULL, "cap1_uplldiv");
	//clkaplldiv = clk_get(NULL, "cap1_eclk_div");
	if (IS_ERR(clkaplldiv)) {
		printk(KERN_ERR "nuc980-cap1:failed to get cap clock source\n");
		ret = PTR_ERR(clkaplldiv);
		return ret;
	}
	clk_set_parent(clkmux, clkaplldiv);
	clk_set_rate(clkcap, video1_freq);



	i32Div=(300000000/video1_freq)-1;
	if(i32Div>0xF) i32Div=0xf;
	__raw_writel((__raw_readl(REG_CLK_DIV2) & ~(0xF<<24) ) | i32Div<<24,REG_CLK_DIV2);

	/* PC0 set to low; PD# */
	ret = gpio_request(CAP1_PD_PIN, "CAP1_PD_PIN");
	if (ret) {
		printk("CAP1_PD_PIN failed ret=%d\n",ret);
		return ret;
	}
	gpio_direction_output(CAP1_PD_PIN,1);
	udelay(100);
	gpio_set_value(CAP1_PD_PIN, 0 );

	/* PE10 set to high; RST# */
	ret = gpio_request(CAP1_RST_PIN, "CAP1_RST_PIN");
	if (ret) {
		printk("CAP1_RST_PIN failed ret=%d\n",ret);
		return ret;
	}
	gpio_direction_output(CAP1_RST_PIN,0);
	udelay(100);
	gpio_set_value(CAP1_RST_PIN, 1 );
	udelay(100);

#if 0
	printk("video1_freq=%d\n",video1_freq);
	printk("GCR_BA+0x090=0x%08x\n",__raw_readl(GCR_BA+0x090));
	printk("GCR_BA+0x094=0x%08x\n",__raw_readl(GCR_BA+0x094));
	printk("GCR_BA+0x09c=0x%08x\n",__raw_readl(GCR_BA+0x09C));
	printk("CLK_HCLKEN=0x%08x\n",__raw_readl(CLK_BA+0x010));
	printk("REG_CLK_DIV2=0x%08x\n",__raw_readl(CLK_BA+0x028));
#endif

	return 0;
}

static int vdi_user=0;
static int nuvoton_vdi1_open(struct file *filp)
{
	struct nuvoton_vin_device* cam=video_drvdata(filp);
	int err = 0;
	ENTRY();
	if (!down_read_trylock(&nuvoton_vin1_dev_lock))
		return -EAGAIN;

	if (wait_for_completion_interruptible(&cam->probe)) {
		up_read(&nuvoton_vin1_dev_lock);
		return -ERESTARTSYS;
	}
	kref_get(&cam->kref);

	if (mutex_lock_interruptible(&cam->open_mutex)) {
		kref_put(&cam->kref, nuvoton_vin1_release_resources);
		up_read(&nuvoton_vin1_dev_lock);
		return -ERESTARTSYS;
	}
	if (cam->users) {
		VDEBUG("Device %s is busy...\n",video_device_node_name(cam->v4ldev));
		VDEBUG("Simultaneous opens are not supported\n");
		if ((filp->f_flags & O_NONBLOCK) || (filp->f_flags & O_NDELAY)) {
			err = -EWOULDBLOCK;
			goto out;
		}
		VDEBUG("A blocking open() has been requested. Wait for the device to be released...\n");
		if (err)
			goto out;
	}

	filp->private_data = cam;
	cam->users=++vdi_user;
	cam->io = IO_NONE;
	cam->stream = STREAM_OFF;
	cam->nbuffers = 0;
	cam->frame_count = 0;
	nuvoton_vin1_empty_framequeues(cam);
	VDEBUG("Video device %s is open\n",video_device_node_name(cam->v4ldev));

out:
	mutex_unlock(&cam->open_mutex);
	if (err)
		kref_put(&cam->kref, nuvoton_vin1_release_resources);
	up_read(&nuvoton_vin1_dev_lock);
	LEAVE();
	return err;

}

static int nuvoton_vdi1_close(struct file *filp)
{

	struct nuvoton_vin_device* cam = video_drvdata(filp);
	ENTRY();
	down_write(&nuvoton_vin1_dev_lock);
	nuvoton_vdi1_disable(cam);
	nuvoton_vin1_release_buffers(cam);
	vdi_user--;
	cam->users=0;
	cam->vpe.PacketEnable=0;
	cam->vpe.PlanarEnable=0;
	cam->type=0;
	cam->stream = STREAM_OFF;
	VDEBUG("Video device %s closed", video_device_node_name(cam->v4ldev));

	kref_put(&cam->kref, nuvoton_vin1_release_resources);
	up_write(&nuvoton_vin1_dev_lock);
	nuvoton_vin1_empty_framequeues(cam);
	wake_up_interruptible(&cam->wait_frame);
	LEAVE();
	return 0;
}

static ssize_t nuvoton_vdi1_read(struct file* filp, char __user * buf, size_t count, loff_t* f_pos)
{
	struct nuvoton_vin_device *cam = video_drvdata(filp);
	struct nuvoton_vin_frame_t* f, * i;
	unsigned long lock_flags;
	long timeout;
	int err = 0;
	ENTRY();
	if (mutex_lock_interruptible(&cam->fileop_mutex)) {
		return -ERESTARTSYS;
	}

	if (cam->io == IO_MMAP) {
		VDEBUG("Close and open the device again to choose the read method\n");
		mutex_unlock(&cam->fileop_mutex);
		return -EBUSY;
	}
	if (cam->io == IO_NONE) {
		if (!nuvoton_vin1_request_buffers(cam, cam->nreadbuffers, IO_READ)) {
			VDEBUG("read() failed, not enough memory\n");
			mutex_unlock(&cam->fileop_mutex);
			return -ENOMEM;
		}
		cam->io = IO_READ;
		cam->stream = STREAM_ON;
		if(cam->nreadbuffers>0) {
			VDEBUG("cam->nreadbuffers=%d\n",cam->nreadbuffers);
			if(cam->vpe.PacketEnable==1) {
				if(cam->frame_current!=NULL)
					__raw_writel(cam->frame_current->pbuf,REG_CAP1_PKTBA0);
				else
					VDEBUG("PacketEnable : cam->frame_current == NULL\n");
			}
			if(cam->vpe.PlanarEnable==1) {
				if(cam->frame_current!=NULL) {
					__raw_writel((unsigned int)cam->frame_current->pbuf,REG_CAP1_YBA);
					__raw_writel(__raw_readl(REG_CAP1_YBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight),REG_CAP1_UBA);
					__raw_writel(__raw_readl(REG_CAP1_UBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight)/2,REG_CAP1_VBA);
				} else
					VDEBUG("PlanarEnable : cam->frame_current == NULL\n");
			}

			VDEBUG("cam->frame_current->bufmem=0x%08x\n",cam->frame_current->bufmem);
			VDEBUG("cam->frame_current->pbuf=0x%08x\n",cam->frame_current->pbuf);
			VDEBUG("cam->vpe.PacketEnable=%d\n",cam->vpe.PacketEnable);
			VDEBUG("cam->vpe.PlanarEnable=%d\n",cam->vpe.PlanarEnable);
		}
		/* Capture engine enable and packet/planar mode enable */
		nuvoton_vdi1_enable(cam);
	}

	if (list_empty(&cam->inqueue)) {
		if (!list_empty(&cam->outqueue))
			nuvoton_vin1_empty_framequeues(cam);
		nuvoton_vin1_queue_unusedframes(cam);
	}

	if (!count) {
		mutex_unlock(&cam->fileop_mutex);
		return 0;
	}

	if (list_empty(&cam->outqueue)) {
		timeout = wait_event_interruptible_timeout
		          ( cam->wait_frame,(!list_empty(&cam->outqueue)),msecs_to_jiffies(NUVOTON_FRAME_TIMEOUT * 1000));
		if (timeout < 0) {
			mutex_unlock(&cam->fileop_mutex);
			return timeout;
		}
	}
	f = list_entry(cam->outqueue.prev, struct nuvoton_vin_frame_t, frame);
	if (copy_to_user(buf, f->bufmem, count)) {
		err = -EFAULT;
		goto exit;
	}
	*f_pos += count;

exit:

	spin_lock_irqsave(&cam->queue_lock, lock_flags);
	list_for_each_entry(i, &cam->outqueue, frame)
	i->state = F_UNUSED;
	INIT_LIST_HEAD(&cam->outqueue);
	spin_unlock_irqrestore(&cam->queue_lock, lock_flags);

	nuvoton_vin1_queue_unusedframes(cam);

	VDEBUG("Frame #%lu, bytes read: %zu\n",(unsigned long)f->buf.index, count);

	mutex_unlock(&cam->fileop_mutex);
	LEAVE();
	return 0;
}

static void nuvoton_vin1_vm_open(struct vm_area_struct* vma)
{
	struct nuvoton_vin_frame_t* f = vma->vm_private_data;
	ENTRY();
	f->vma_use_count++;
}


static void nuvoton_vin1_vm_close(struct vm_area_struct* vma)
{
	/* NOTE: buffers are not freed here */
	struct nuvoton_vin_frame_t* f = vma->vm_private_data;
	ENTRY();
	f->vma_use_count--;
	LEAVE();
}


static const struct vm_operations_struct nuvoton_vin1_vm_ops = {
	.open = nuvoton_vin1_vm_open,
	.close = nuvoton_vin1_vm_close,
};

static int nuvoton_vdi1_mmap(struct file* filp, struct vm_area_struct *vma)
{
	struct nuvoton_vin_device *cam = video_drvdata(filp);
	unsigned long size = vma->vm_end - vma->vm_start,start = vma->vm_start;
	void *pos;
	u32 i;
	ENTRY();
	//if (mutex_lock_interruptible(&cam->fileop_mutex))
	//  return -ERESTARTSYS;
	if (!(vma->vm_flags & (VM_WRITE | VM_READ))) {
		mutex_unlock(&cam->fileop_mutex);
		return -EACCES;
	}

	if (cam->io != IO_MMAP ||
	    size != PAGE_ALIGN(cam->frame[0].buf.length)) {
		mutex_unlock(&cam->fileop_mutex);
		return -EINVAL;
	}

	for (i = 0; i < cam->nbuffers; i++) {
		if ((cam->frame[i].buf.m.offset>>PAGE_SHIFT) == vma->vm_pgoff)
			break;
	}
	if (i == cam->nbuffers) {
		mutex_unlock(&cam->fileop_mutex);
		return -EINVAL;
	}

	vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
	pos = cam->frame[i].bufmem;
	while (size > 0) { /* size is page-aligned */
		if (vm_insert_page(vma, start, vmalloc_to_page(pos))) {
			mutex_unlock(&cam->fileop_mutex);
			//printk("4444\n");
			return -EAGAIN;
		}
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	vma->vm_ops = &nuvoton_vin1_vm_ops;
	vma->vm_private_data = &cam->frame[i];
	nuvoton_vin1_vm_open(vma);

	mutex_unlock(&cam->fileop_mutex);
	LEAVE();
	return 0;
}

/* ISR for buffer handling                   *
 * for nuvoton sensor interface              *
 *                                           */
static int cap1_cnt=0;
static irqreturn_t nuvoton_vdi1_isr(int irq, void *priv)
{
	struct nuvoton_vin_device* cam=priv;
	struct nuvoton_vin_frame_t** f=NULL;
	ENTRY();
	//printk("VDIN1 ISR status=0x%08x\n",__raw_readl(REG_CAP1_INT));
	//printk("REG_CAP1_CTL=0x%08x\n",__raw_readl(REG_CAP1_CTL));
	//printk("REG_CAP1_CURADDRY=0x%08x\n",__raw_readl(REG_CAP1_CURADDRY));
	//printk("REG_CAP1_CURADDRU=0x%08x\n",__raw_readl(REG_CAP1_CURADDRU));
	//printk("REG_CAP1_CURADDRV=0x%08x\n",__raw_readl(REG_CAP1_CURADDRV));
	//printk("REG_CAP1_YBA=0x%08x\n",__raw_readl(REG_CAP1_YBA));
	//printk("REG_CAP1_UBA=0x%08x\n",__raw_readl(REG_CAP1_UBA));
	//printk("REG_CAP1_VBA=0x%08x\n",__raw_readl(REG_CAP1_VBA));
	if(cap1_cnt==1) {
		cap1_cnt=0;
		__raw_writel(__raw_readl(REG_CAP1_INT),REG_CAP1_INT);
		LEAVE();
		return IRQ_NONE;
	}

	f  = &cam->frame_current;
//      if (cam->stream == STREAM_OFF || list_empty(&cam->inqueue)) {
	if (cam->stream == STREAM_OFF || list_is_last(&(*f)->frame,&cam->inqueue)) {
		if(cam->stream == STREAM_ON && cam->type==V4L2_BUF_TYPE_VIDEO_OVERLAY) {
#if 0
			__raw_writel(cam->frame[0].pbuf,NUC970_VA_LCD+REG_LCM_VA_BADDR0);
			__raw_writel(cam->frame[0].pbuf,REG_CAP1_PKTBA0);
			__raw_writel(__raw_readl(REG_CAP1_CTL) | CAP_CTL_UPDATE,REG_CAP1_CTL);
			__raw_writel((__raw_readl(REG_CAP1_INT) & ~0x10000) ,REG_CAP1_INT);        /* Disable CAP Interrupt */
#endif
		} else {
			//printk("wake_up_interruptible start\n");
			wake_up_interruptible(&cam->wait_frame);
			//printk("wake_up_interruptible end\n");
		}
	}
	spin_lock(&cam->queue_lock );
	if((*f)->state == F_QUEUED && !list_is_last(&(*f)->frame,&cam->inqueue)) {
		list_move_tail(&(*f)->frame, &cam->outqueue);
	}
	if (!list_empty(&cam->inqueue)) {
		(*f) = list_entry(cam->inqueue.next,struct nuvoton_vin_frame_t,frame);
		if(cam->vpe.PacketEnable==1) {
			/* Setting packet buffer start address */
			__raw_writel((*f)->pbuf,REG_CAP1_PKTBA0);
			if(cam->sensor.bothenable==1) {
				struct nuvoton_vin_sensor* s = &cam->sensor;
				struct v4l2_rect* bounds = &(s->cropcap.bounds);
				u32 size = bounds->height*bounds->height*2;
				__raw_writel((*f)->pbuf+size,REG_CAP1_YBA);
				__raw_writel(__raw_readl(REG_CAP1_YBA)+(cam->vpe.PlanarHeight*cam->vpe.PlanarWidth),REG_CAP1_UBA);
				__raw_writel(__raw_readl(REG_CAP1_UBA)+(cam->vpe.PlanarHeight*cam->vpe.PlanarWidth)/2,REG_CAP1_VBA);
			}
		} else if(cam->vpe.PlanarEnable==1) {
			/* Setting planar buffer Y address, U address, V address */
			__raw_writel((unsigned int)(*f)->pbuf,REG_CAP1_YBA);
			__raw_writel(__raw_readl(REG_CAP1_YBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight),REG_CAP1_UBA);
			__raw_writel(__raw_readl(REG_CAP1_UBA)+(cam->vpe.PlanarWidth*cam->vpe.PlanarHeight)/2,REG_CAP1_VBA);
		}
		/* Update New frame */
		__raw_writel(__raw_readl(REG_CAP1_CTL) | CAP_CTL_UPDATE,REG_CAP1_CTL);
		cap1_cnt++;
		spin_unlock(&cam->queue_lock);
		wake_up_interruptible(&cam->wait_frame);
	}
	__raw_writel(__raw_readl(REG_CAP1_INT),REG_CAP1_INT);
	LEAVE();
	return IRQ_NONE;
}



static struct v4l2_file_operations nuvoton_vdi1_fops = {
	.owner              = THIS_MODULE,
	.open               = nuvoton_vdi1_open,
	.release            = nuvoton_vdi1_close,
	.read               = nuvoton_vdi1_read,
	//.ioctl            = video_ioctl2,         /* V4L2 ioctl handler */
	.unlocked_ioctl     = video_ioctl2,         /* V4L2 ioctl handler */
	.mmap               = nuvoton_vdi1_mmap,
};

/* ---- Initialization v4l2_file_operations  ----*/
#ifndef CONFIG_USE_OF
#if defined(CONFIG_SENSOR1_OV7725)
extern int nuvoton_vin1_probe_ov7725(struct nuvoton_vin_device* cam);
#elif defined(CONFIG_SENSOR1_OV5640)
extern int nuvoton_vin1_probe_ov5640(struct nuvoton_vin_device* cam);
#elif defined(CONFIG_SENSOR1_NT99141)
extern int nuvoton_vin1_probe_nt99141(struct nuvoton_vin_device* cam);
#elif defined(CONFIG_SENSOR1_NT99050)
extern int nuvoton_vin1_probe_nt99050(struct nuvoton_vin_device* cam);
#elif defined(CONFIG_SENSOR1_TW9912)
extern int nuvoton_vin1_probe_tw9912(struct nuvoton_vin_device* cam);
#elif defined(CONFIG_SENSOR1_GC0308)
extern int nuvoton_vin1_probe_gc0308(struct nuvoton_vin_device* cam);
#endif
#else
extern int nuvoton_vin1_probe_ov7725(struct nuvoton_vin_device* cam);
extern int nuvoton_vin1_probe_ov5640(struct nuvoton_vin_device* cam);
extern int nuvoton_vin1_probe_nt99141(struct nuvoton_vin_device* cam);
extern int nuvoton_vin1_probe_nt99050(struct nuvoton_vin_device* cam);
extern int nuvoton_vin1_probe_tw9912(struct nuvoton_vin_device* cam);
extern int nuvoton_vin1_probe_gc0308(struct nuvoton_vin_device* cam);
#endif
int nuvoton_vdi1_device_register(struct platform_device *pdev)
{
	int ret;
	struct nuvoton_vin_device* cam;
	int err = 0;
	ENTRY();

	if (!(cam = kzalloc(sizeof(struct nuvoton_vin_device), GFP_KERNEL)))
		return -ENOMEM;
	if (!(cam->control_buffer = kzalloc(4, GFP_KERNEL))) {
		VDEBUG("kmalloc() failed\n");
		err = -ENOMEM;
		goto fail;
	}
	if (!(cam->frame = kzalloc(sizeof(struct nuvoton_vin_frame_t)*CONFIG_VIN1_MAX_FRAME_BUFFER, GFP_KERNEL)))
		return -ENOMEM;
	if (!(cam->vir_addr = (u32 **)kzalloc(sizeof(u32 *)*CONFIG_VIN1_MAX_FRAME_BUFFER, GFP_KERNEL)))
		return -ENOMEM;
	if (!(cam->phy_addr = kzalloc(sizeof(dma_addr_t)*CONFIG_VIN1_MAX_FRAME_BUFFER, GFP_KERNEL)))
		return -ENOMEM;
	if (!(cam->v4ldev = video_device_alloc())) {
		VDEBUG("video_device_alloc() failed\n");
		err = -ENOMEM;
		goto fail;
	}
	err=capture1_init(cam); /* Set capture init for nuvoton sensor interface */
	VDEBUG("capture1_init().");

	//for sensor init
#ifndef CONFIG_USE_OF
#if defined(CONFIG_SENSOR1_OV7725)
	err=nuvoton_vin1_probe_ov7725(cam); //sensor probe;
#elif defined(CONFIG_SENSOR1_OV5640)
	err=nuvoton_vin1_probe_ov5640(cam); //sensor probe;
#elif defined(CONFIG_SENSOR1_NT99141)
	err=nuvoton_vin1_probe_nt99141(cam); //sensor probe;
#elif defined(CONFIG_SENSOR1_NT99050)
	err=nuvoton_vin1_probe_nt99050(cam); //sensor probe;
#elif defined(CONFIG_SENSOR1_TW9912)
	err=nuvoton_vin1_probe_tw9912(cam);//sensor probe;
#elif defined(CONFIG_SENSOR1_GC0308)
	err=nuvoton_vin1_probe_gc0308(cam);//sensor probe;
#endif
	if(err<0) {
		gpio_free(CAP1_PD_PIN);
		gpio_free(CAP1_RST_PIN);
		VDEBUG("Initialization failed. I will retry on open().");
		return err;
	}
#else
	if(sensor1_model==SENSOR_OV7725)
		err=nuvoton_vin1_probe_ov7725(cam); //sensor probe;
	else if(sensor1_model==SENSOR_OV5640)
		err=nuvoton_vin1_probe_ov5640(cam); //sensor probe;
	else if(sensor1_model==SENSOR_NT99141)
		err=nuvoton_vin1_probe_nt99141(cam); //sensor probe;
	else if(sensor1_model==SENSOR_NT99050)
		err=nuvoton_vin1_probe_nt99050(cam); //sensor probe;
	else if(sensor1_model==SENSOR_TW9912)
		err=nuvoton_vin1_probe_tw9912(cam); //sensor probe;
	else if(sensor1_model==SENSOR_GC0308) {
		err=nuvoton_vin1_probe_gc0308(cam); //sensor probe;
	}

	if(err<0) {
		gpio_free(CAP1_PD_PIN);
		gpio_free(CAP1_RST_PIN);
		VDEBUG("Initialization failed. I will retry on open().");
		return -EAGAIN;
	}
#endif

	{
		int j;
		int mul=1;
		struct v4l2_rect* defrect = &(cam->sensor.cropcap.defrect);
		if(cam->sensor.bothenable==1) mul=2;
		for(j=0; j<CONFIG_VIN1_MAX_FRAME_BUFFER; j++)
			if((cam->vir_addr[j] = dma_alloc_writecombine(NULL,
			                       PAGE_ALIGN(defrect->width*defrect->height*2*mul),
			                       &cam->phy_addr[j],
			                       GFP_KERNEL))==NULL) {
				printk("dma_alloc_writecombine failed\n");
				return -EAGAIN;
			}
	}
	/* INIT */
	mutex_init(&cam->open_mutex);
	mutex_init(&cam->fileop_mutex);
	init_waitqueue_head(&cam->wait_frame);
	init_waitqueue_head(&cam->wait_stream);
	cam->nreadbuffers = 2;

	strcpy(cam->v4ldev->name, "NUVOTON Camera1 Interface");
	cam->stream = STREAM_OFF;
	//cam->v4ldev->current_norm = V4L2_STD_NTSC_M;
	cam->v4ldev->fops       = &nuvoton_vdi1_fops;
	cam->v4ldev->release        = video_device_release;
	cam->v4ldev->tvnorms        = V4L2_STD_525_60;
	cam->v4ldev->ioctl_ops = &nuvoton_vdi1_ioctl_ops; /* for V4L2 ioctl handler */
	cam->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cam->frame_current = NULL;

	ret=v4l2_device_register(&pdev->dev,&cam->v4l2_dev);
	if (ret) {
		printk("Unable to register v4l2 device\n");
		v4l2_err(pdev->dev.driver,
		         "Unable to register v4l2 device\n");
		return ret;
	}
	cam->v4ldev->v4l2_dev =(struct v4l2_device *) &cam->v4l2_dev;
	video_set_drvdata(cam->v4ldev, cam);
	platform_set_drvdata(pdev, cam);
	init_completion(&cam->probe);

	err = video_register_device(cam->v4ldev, VFL_TYPE_GRABBER,
	                            -1);

	if (err) {
		VDEBUG("V4L2 device registration failed\n");
		if (err == -ENFILE)
			VDEBUG("Free /dev/videoX node not found\n");
		complete_all(&cam->probe);
		goto fail;
	}

	/* Setting Interrupt(IRQ) for nuvoton sensor interface  */
	err = request_irq(IRQ_CAP1, nuvoton_vdi1_isr, IRQF_NO_SUSPEND|IRQF_SHARED, "camera sensor 1", cam);
	if(err < 0) {
		VDEBUG("Interrupt(IRQ_CAP1) setup failed\n");
		goto fail;
	} else
		VDEBUG("%s, main sensor isr is created\n", __func__);
	VDEBUG("V4L2 device registered as %s\n",video_device_node_name(cam->v4ldev));
	kref_init(&cam->kref);
	complete_all(&cam->probe);
	return 0;
fail:
	if (cam) {
		kfree(cam->control_buffer);
		if (cam->v4ldev)
			video_device_release(cam->v4ldev);
		kfree(cam);
	}
	return err;
}

/*****************************************************************************/ //for sensor
void nuvoton_vin1_attach_sensor(struct nuvoton_vin_device* cam, struct nuvoton_vin_sensor* sensor)
{
	memcpy(&cam->sensor, sensor, sizeof(struct nuvoton_vin_sensor));
}
/*****************************************************************************/


/* This routine allocates from 1 to n_devs drivers.
The real maximum number of virtual drivers will depend on how many drivers   will succeed.
This is limited to the maximum number of devices that   videodev supports.
Since there are 64 minors for video grabbers, this is   currently the theoretical maximum limit.
However, a further limit does   exist at videodev that forbids any driver to register more than 32 video   grabbers.
Max number of video_devices can be registered at the same time: 32 */
static int nuvoton_cap1_device_probe(struct platform_device *pdev)
{
	int ret = 0;
#ifndef CONFIG_USE_OF
	struct pinctrl *pinctrl;
#endif
	ENTRY();
	printk("%s - pdev = %s\n", __func__, pdev->name);

#ifndef CONFIG_USE_OF
	video1_freq = CONFIG_VIDEO1_FREQ;


#if 0
	__raw_writel((__raw_readl(GCR_BA+0x090)&0x00000000)|0x77777777,(GCR_BA+0x090));
	__raw_writel((__raw_readl(GCR_BA+0x094)&0xFFF0F000)|0x00070077,(GCR_BA+0x094));
	__raw_writel((__raw_readl(GCR_BA+0x09C)&0xFFFFF0FF)|0x00000700,(GCR_BA+0x09C));
#else
	pinctrl = devm_pinctrl_get_select(&pdev->dev, "vcap1");
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "unable to reserve pin\n");
		ret = PTR_ERR(pinctrl);
		return ret;
	}
#endif

#else
	{
		const char *pstr;

		of_property_read_u32_array(pdev->dev.of_node,"frequency", &video1_freq,1);
		of_property_read_string(pdev->dev.of_node,"model",&pstr);
		if(pstr[0]=='o' && pstr[1]=='v' && pstr[2]=='7' && pstr[3]=='7' && pstr[4]=='2' && pstr[5]=='5' ) {
			sensor1_model = 0;
		} else if(pstr[0]=='n' && pstr[1]=='t' && pstr[2]=='9' && pstr[3]=='9' && pstr[4]=='1' && pstr[5]=='4' && pstr[6]=='1' ) {
			sensor1_model = 1;
		} else if(pstr[0]=='n' && pstr[1]=='t' && pstr[2]=='9' && pstr[3]=='9' && pstr[4]=='0' && pstr[5]=='5' && pstr[6]=='0' ) {
			sensor1_model = 2;
		} else if(pstr[0]=='o' && pstr[1]=='v' && pstr[2]=='5' && pstr[3]=='6' && pstr[4]=='4' && pstr[5]=='0' ) {
			sensor1_model = 3;
		} else if(pstr[0]=='t' && pstr[1]=='w' && pstr[2]=='9' && pstr[3]=='9' && pstr[4]=='1' && pstr[5]=='2') {
			sensor1_model = 4;
		} else if(pstr[0]=='g' && pstr[1]=='c' && pstr[2]=='0' && pstr[3]=='3' && pstr[4]=='0' && pstr[5]=='8') {
			sensor1_model = 5;
		}

		//of_property_read_string(pdev->dev.of_node,"powerdown-pin",&pstr1);
	}
#endif

	ret = nuvoton_vdi1_device_register(pdev);
	if(ret) {
		printk("%s video%d, main sensor registeration fail.\n", __func__,1);
		ret = -EPROBE_DEFER;
		goto out;
	} else
		printk("%s video%d, main sensor registeration ok.\n", __func__,1);
out:
	LEAVE();
	return ret;
}

static int nuvoton_cap1_device_remove(struct platform_device *pdev)
{
	ENTRY();
	LEAVE();
	return 0;
}

static int nuvoton_cap1_device_resume(struct platform_device *pdev)
{
	struct nuvoton_vin_device* cam = platform_get_drvdata(pdev);
	ENTRY();
	capture1_init(0);
#ifndef CONFIG_USE_OF
#if defined(CONFIG_SENSOR1_OV7725)
	nuvoton_vin1_probe_ov7725(cam);
#elif defined(CONFIG_SENSOR1_OV5640)
	nuvoton_vin1_probe_ov5640(cam);
#elif defined(CONFIG_SENSOR1_NT99141)
	nuvoton_vin1_probe_nt99141(cam);
#elif defined(CONFIG_SENSOR1_NT99050)
	nuvoton_vin1_probe_nt99050(cam);
#elif defined(CONFIG_SENSOR1_TW9912)
	nuvoton_vin1_probe_tw9912(cam);
#elif defined(CONFIG_SENSOR1_GC0308)
	nuvoton_vin1_probe_gc0308(cam);
#endif
#else
	if(sensor1_model==SENSOR_OV7725)
		nuvoton_vin1_probe_ov7725(cam);
	else if(sensor1_model==SENSOR_OV5640)
		nuvoton_vin1_probe_ov5640(cam);
	else if(sensor1_model==SENSOR_NT99141)
		nuvoton_vin1_probe_nt99141(cam);
	else if(sensor1_model==SENSOR_NT99050)
		nuvoton_vin1_probe_nt99050(cam);
	else if(sensor1_model==SENSOR_TW9912)
		nuvoton_vin1_probe_tw9912(cam);
	else if(sensor1_model==SENSOR_GC0308)
		nuvoton_vin1_probe_gc0308(cam);
#endif
	if(cam->CtlReg==1)
		__raw_writel(__raw_readl(REG_CAP1_CTL)|0x1,REG_CAP1_CTL);
	LEAVE();
	return 0;
}

static int nuvoton_cap1_device_suspend(struct platform_device *pdev,pm_message_t state)
{
	struct nuvoton_vin_device* cam = platform_get_drvdata(pdev);
	ENTRY();
	if(__raw_readl(REG_CAP1_CTL)&0x1) {
		__raw_writel(__raw_readl(REG_CAP1_CTL)|(1<<16),REG_CAP1_CTL);
		while(__raw_readl(REG_CAP1_CTL)&0x1);
		cam->CtlReg=1;
	} else {
		cam->CtlReg=0;
	}
	capture1_uninit();
	LEAVE();
	return 0;
}

static struct platform_device_id nuc980_cap1_driver_ids[] = {
	{ "nuc980-videoin1", 0 },
	{ },
};

static const struct of_device_id nuc980_cap1_of_match[] = {
	{ .compatible = "nuvoton,nuc980-cap1" },
	{},
};
static struct platform_driver nuc980_cap1_driver = {
	.probe  = nuvoton_cap1_device_probe,
	.remove     = nuvoton_cap1_device_remove,
	.resume   = nuvoton_cap1_device_resume,
	.suspend  = nuvoton_cap1_device_suspend,
	.driver     = {
		.name   = "nuc980-cap1",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nuc980_cap1_of_match),
	},
	.id_table   = nuc980_cap1_driver_ids,
};

module_platform_driver(nuc980_cap1_driver);
MODULE_DESCRIPTION("NUVOTON sensor interface");
MODULE_AUTHOR("SCHung");
MODULE_LICENSE("Dual BSD/GPL");
