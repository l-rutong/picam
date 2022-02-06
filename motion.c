/* motion.h */
/*
 * (c) 2015 Dickon Hood <dickon@fluff.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * Motion detection.
 *
 * This is the fun bit.  The basic principle is simple: take the motion
 * vector map from the encoder, and see if it exceeds our thresholds,
 * triggering recording.  To do this, we can either take a static
 * sensitivity (across the whole frame), or the filename of an 8bpp
 * greyscale bitmap; either way, transform this into a 16b/mb 'heatmap',
 * which we then test against.  If more than mctx.threshold macroblocks
 * exceed their sensitivity ratings, trigger a recording.
 *
 * Simple.
 *
 * To speed things up a bit, we pre-square the heatmap, and don't bother
 * sqrt()ing the magnitude vectors to find the actual hypoteneuse.  This
 * turns the critical section into 14,400 byte loads and multiplies, 7,200
 * half-word loads, and a bit of trivial maths.  Or would be, if I wrote it
 * in assembly; the compiler does a very poor job.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/statvfs.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <alsa/asoundlib.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/mathematics.h>
#include <getopt.h>

#include "ilclient.h"

#include "motion.h"
#include <png.h>

static void *motionstart(void *);



struct motvec {
	int8_t			dx;
	int8_t			dy;
	uint16_t		sad;
};



static struct {
	pthread_cond_t		cond;
	pthread_mutex_t		lock;
	int			width, height;
	uint16_t		*map;
	struct motvec		*vectors;
	int			threshold;
	pthread_t		detectionthread;
#define FLAGS_MOVEMENT		(1<<0)
#define FLAGS_MOTMONITOR	(1<<1)
	int			flags;
	void 			(*eventcb)(int motion, enum movementevents);
	char			*pngfn;
	int				window_size;
	int				*filter_window;
} mctx;



static int readmap(char *mf)
{
	FILE *fd;
	uint8_t header[8];
	png_structp png;
	png_infop info, end;
	png_bytep *rows;
	int i, j;

	fd = fopen(mf, "rb");
	if (!fd)
		return -1;
	fread(header, 1, sizeof(header), fd);
	if (png_sig_cmp(header, 0, sizeof(header)))
		return -1;
	png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (!png)
		return -1;
	info = png_create_info_struct(png);
	if (!info)
		return -1;
	end = png_create_info_struct(png);
	if (!end)
		return -1;
	png_init_io(png, fd);
	png_set_sig_bytes(png, sizeof(header));
	png_read_png(png, info, PNG_TRANSFORM_STRIP_16 |
		PNG_TRANSFORM_STRIP_ALPHA | PNG_TRANSFORM_PACKING, NULL);
	rows = png_get_rows(png, info);
printf("Mapfile: \n");
	for (i = 0; i < mctx.height; i++) {
		uint8_t *r = rows[i];
		for (j = 0; j < mctx.width-1; j++) {
			mctx.map[i*(mctx.width) + j] =
				(uint16_t) (r[j] * r[j]);
printf("%d ", mctx.map[i*(mctx.width) + j]);
		}
		mctx.map[(i+1)*(mctx.width)] = 65535;
printf("\n");
	}
	png_destroy_read_struct(&png, &info, &end);
	fclose(fd);

	return 0;
}



int initmotion(int width, int height, char *map, int sens, int thresh, int window_size,
	void(*eventcb)(int, enum movementevents))
{
	int rows, cols;
	int x, y;
	pthread_attr_t detach;

	memset(&mctx, 0, sizeof(mctx));

	mctx.height = rows = (height + 15) / 16;
	mctx.width = cols = ((width + 15) / 16) + 1;
	mctx.map = (uint16_t *) malloc((sizeof(uint16_t)) * (cols+1)*rows);
	mctx.threshold = thresh; //(rows * cols * thresh) / 100;
	mctx.pngfn = "/run/shm/hls/dump.png";
	mctx.flags = FLAGS_MOTMONITOR;
	mctx.window_size = window_size;
	mctx.filter_window = (int *) calloc(window_size, sizeof(int));

	printf("PNG filename: %s\n", mctx.pngfn);
	if (map) {
		printf("Reading mapfile %s\n", map);
		if (readmap(map) != 0) {
			printf("Failed to read mapfile: %s\n",
				strerror(errno));
			return -1;
		}
	} else {
		sens = sens * sens;
		if (sens > 65535)
			sens = 65535;
		for (y = 0; y < rows; y++) {
			for (x = 0; x < cols-1; x++) {
				mctx.map[y*cols + x] = (uint16_t) sens;
			}
			mctx.map[y*cols + cols] = 65535;
		}
	}

	mctx.eventcb = eventcb;

	pthread_mutex_init(&mctx.lock, NULL);
	pthread_cond_init(&mctx.cond, NULL);
	pthread_attr_init(&detach);
	pthread_attr_setdetachstate(&detach, PTHREAD_CREATE_DETACHED);	
	pthread_create(&mctx.detectionthread, &detach, motionstart, NULL);

	return 0;
}

int ma_filter(int t)
{
	static int i = 0;
	static int sum = 0;
	int n, nf;
	int j,k;
	int head_avg = 0;

	int ot = t;
	int nmblk = mctx.width * mctx.height;

	// concider motion value as noise is more than 1/4 of the micro blocks are moved
	if(t > (nmblk / 4)) t = mctx.threshold;

	// calculate noise floor
	// big motion value should not be considered as noise
	n = t;
	if(n > mctx.threshold * 4) n = mctx.threshold * 4;
	sum+=n;

	n = mctx.filter_window[i];
	if(n > mctx.threshold * 4) n = mctx.threshold * 4;
	sum-=n;

	nf = sum / mctx.window_size;

	// put latest value
	mctx.filter_window[i]=t;

	// printf("nf(%d) ", nf);

	// calculate moving avg of recent motion value
	for(j = 0; j < 12; j++) {
		k = i - j;
		if(k < 0) k = mctx.window_size + i - j;
		head_avg += mctx.filter_window[k];
	}
	t = head_avg / 12;

	// filter windows is a ring buffer
	i++;	
	if(i >= mctx.window_size) i = 0;

	// deduct by noise floor
	if (t > nf) t = t - nf;
	else t = 0;

printf("\r%4d(%4d -%3d) / %d (%d).", t, ot, nf, mctx.threshold, nmblk);
fflush(stdout);

	return t;
}

void dumppng(struct motvec *v)
{
	int i, j;
	static int fnum = 0;
	static png_bytep *rows;
	FILE *fd;
	png_structp png;
	png_infop info;
	static char fn[256];

	if (fnum == 0) {
		rows = malloc(mctx.height * sizeof(png_bytep));
		for (i = 0; i < mctx.height; i++) {
			rows[i] = (png_bytep) malloc(mctx.width);
		}
	}

	for (i = 0; i < mctx.height; i++) {
		for (j = 0; j < mctx.width; j++) {
			uint8_t *t;
			struct motvec *tv;
			t = (uint8_t *) rows[i];
			tv = &v[(i * mctx.width) + j];
			t[j] = sqrt((double)
				((tv->dx * tv->dx) + (tv->dy * tv->dy)));
		}
	}

	png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	info = png_create_info_struct(png);
	sprintf(fn, mctx.pngfn, fnum);
	fd = fopen(fn, "wb");
	png_init_io(png, fd);
	png_set_compression_level(png, 0);
	png_set_IHDR(png, info, mctx.width-1, mctx.height, 8,
		PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
		PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
	png_set_rows(png, info, rows);
	png_write_png(png, info, PNG_TRANSFORM_IDENTITY, NULL);
	png_write_end(png, NULL);
	png_destroy_write_struct(&png, &info);
	fclose(fd);
//	sprintf(fn, "vo/img%05d.raw", fnum);
//	fd = fopen(fn, "wb");
//	fwrite(v, mctx.width * mctx.height, sizeof(struct motvec), fd);
//	fclose(fd);
	fnum++;
}



static void lookformotion(struct motvec *v)
{
	int i;
	int n;
	int t;
	int ft;

	n = (mctx.width) * mctx.height;

	for (i = t = 0; i < n /* && t < mctx.threshold */; i++) {
		if (mctx.map[i] <
			((v[i].dx * v[i].dx) + (v[i].dy * v[i].dy))) {
			t++;
		}
	}

	if (mctx.pngfn)
		dumppng(v);

	ft = ma_filter(t);

	if (ft >= mctx.threshold) { 
		mctx.eventcb(ft, movement);
	} 
}



static void *motionstart(void *args)
{
	struct motvec *tv;

	while (1) {
		pthread_mutex_lock(&mctx.lock);
		pthread_cond_wait(&mctx.cond, &mctx.lock);
		tv = mctx.vectors;
		mctx.vectors = NULL;
		pthread_mutex_unlock(&mctx.lock);
		lookformotion(tv);
		av_free(tv);
	}

	return NULL; /* to shut the compiler up */
}



void findmotion(OMX_BUFFERHEADERTYPE *out)
{
	uint8_t		*tmpbuf;
	tmpbuf = NULL;
	tmpbuf = av_realloc(tmpbuf, out->nFilledLen);
	memcpy(tmpbuf, out->pBuffer, out->nFilledLen);
	pthread_mutex_lock(&mctx.lock);
	if (mctx.vectors)
		av_free(mctx.vectors);
	mctx.vectors = (struct motvec *) tmpbuf;
	pthread_cond_signal(&mctx.cond);
	pthread_mutex_unlock(&mctx.lock);
}