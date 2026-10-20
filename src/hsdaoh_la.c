/*
 * hsdaoh - High Speed Data Acquisition over MS213x USB3 HDMI capture sticks
 *
 * hsdaoh_la: 8 or 4 channel logic analyzer front end.
 *
 * The device streams samples continuously: the RP2350 'la8' firmware 8-bit
 * samples, two per 16-bit link word (first sample in the high byte, stream
 * format PIO_8BIT); the 'la4' firmware 4-bit samples, four per word (first
 * sample in bits 15:12, format PIO_4BIT). The channel count follows the
 * stream format. This program scans the stream for a trigger condition and
 * saves the samples around the trigger to a file (.sr for PulseView/sigrok,
 * .vcd, or raw .bin; 1 byte per sample, channels in the low bits).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define _FILE_OFFSET_BITS 64

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <time.h>
#include <pthread.h>

#ifndef _WIN32
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#else
#include <windows.h>
#include <getopt.h>
#ifndef usleep
#define usleep(t) Sleep((t)/1000)
#endif
#endif

#include "hsdaoh.h"

#define LA_CHANNELS		8	/* most channels a stream can have (la8) */
#define DEFAULT_POST		1000000ULL
#define DEFAULT_OUTFILE		"capture.sr"
#define SR_MAX_DATA		0xFFFF0000ULL	/* .sr is a plain (non-zip64) zip */

/* ------------------------------------------------------------------ CRC32 */

static uint32_t crc32_tab[4][256];

static void crc32_init(void)
{
	for (uint32_t i = 0; i < 256; i++) {
		uint32_t c = i;
		for (int k = 0; k < 8; k++)
			c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
		crc32_tab[0][i] = c;
	}
	for (uint32_t i = 0; i < 256; i++)
		for (int t = 1; t < 4; t++)
			crc32_tab[t][i] = crc32_tab[0][crc32_tab[t-1][i] & 0xff] ^ (crc32_tab[t-1][i] >> 8);
}

/* slicing-by-4; crc is the running value (start with 0) */
static uint32_t crc32_update(uint32_t crc, const uint8_t *p, size_t n)
{
	crc = ~crc;
	while (n >= 4) {
		crc ^= (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
		crc = crc32_tab[3][crc & 0xff] ^ crc32_tab[2][(crc >> 8) & 0xff] ^
		      crc32_tab[1][(crc >> 16) & 0xff] ^ crc32_tab[0][crc >> 24];
		p += 4;
		n -= 4;
	}
	while (n--)
		crc = crc32_tab[0][(crc ^ *p++) & 0xff] ^ (crc >> 8);
	return ~crc;
}

/* ------------------------------------------------------------------ trigger */

typedef struct {
	uint8_t tbl[65536];	/* tbl[(prev << 8) | cur] != 0: trigger fires on 'cur' */
	char desc[128];
	bool immediate;
	uint8_t used;		/* channels the trigger looks at */
} trigger_t;

/*
 * Trigger spec: comma separated "channel:condition" terms, all must be true.
 *   channel   0..7 (D0..D7 = GP0..GP7 on the la8 firmware; D0..D3 on la4),
 *             'D' prefix allowed
 *   condition 0 = low, 1 = high, r = rising edge, f = falling edge,
 *             c = any edge (change)
 * Example: "0:r,3:1" fires on a rising edge of D0 while D3 is high.
 * An empty spec triggers immediately.
 */
static int trigger_parse(trigger_t *t, const char *spec)
{
	uint8_t lvl_mask = 0, lvl_val = 0, rise = 0, fall = 0, chg = 0, used = 0;
	char buf[256];

	memset(t, 0, sizeof(*t));
	if (!spec || !*spec) {
		memset(t->tbl, 1, sizeof(t->tbl));
		t->immediate = true;
		snprintf(t->desc, sizeof(t->desc), "immediate");
		return 0;
	}

	snprintf(buf, sizeof(buf), "%s", spec);
	for (char *tok = strtok(buf, ","); tok; tok = strtok(NULL, ",")) {
		while (isspace((unsigned char)*tok))
			tok++;
		if (*tok == 'D' || *tok == 'd')
			tok++;
		char *colon = strchr(tok, ':');
		if (!colon || colon == tok || !colon[1] || (colon[2] && !isspace((unsigned char)colon[2]))) {
			fprintf(stderr, "Bad trigger term '%s' (expected e.g. 0:r)\n", tok);
			return -1;
		}
		char *end;
		long ch = strtol(tok, &end, 10);
		if (end != colon || ch < 0 || ch >= LA_CHANNELS) {
			fprintf(stderr, "Bad trigger channel in '%s' (0..%d)\n", tok, LA_CHANNELS - 1);
			return -1;
		}
		uint8_t bit = 1u << ch;
		if (used & bit) {
			fprintf(stderr, "Channel %ld used twice in trigger\n", ch);
			return -1;
		}
		used |= bit;
		switch (tolower((unsigned char)colon[1])) {
		case '0': lvl_mask |= bit; break;
		case '1': lvl_mask |= bit; lvl_val |= bit; break;
		case 'r': rise |= bit; break;
		case 'f': fall |= bit; break;
		case 'c': chg |= bit; break;
		default:
			fprintf(stderr, "Bad trigger condition in '%s' (0, 1, r, f or c)\n", tok);
			return -1;
		}
	}

	for (unsigned p = 0; p < 256; p++) {
		for (unsigned c = 0; c < 256; c++) {
			bool ok = ((c & lvl_mask) == lvl_val) &&
				  ((~p & c & rise) == rise) &&
				  ((p & ~c & fall) == fall) &&
				  (((p ^ c) & chg) == chg);
			t->tbl[(p << 8) | c] = ok;
		}
	}
	snprintf(t->desc, sizeof(t->desc), "%s", spec);
	t->used = used;
	return 0;
}

/* ------------------------------------------------------------------ file output */

/*
 * Captures are first collected in RAM and written to disk afterwards by a
 * separate thread, so the disk speed never limits the capture.
 */

enum { OUT_BIN, OUT_SR, OUT_VCD };

static int out_type_from_name(const char *path)
{
	const char *dot = strrchr(path, '.');
	if (dot && !strcmp(dot, ".sr"))
		return OUT_SR;
	if (dot && !strcmp(dot, ".vcd"))
		return OUT_VCD;
	return OUT_BIN;
}

static void put16(FILE *f, uint16_t v) { fputc(v & 0xff, f); fputc(v >> 8, f); }
static void put32(FILE *f, uint32_t v) { put16(f, v & 0xffff); put16(f, v >> 16); }

static int write_all(FILE *f, const uint8_t *d, uint64_t n)
{
	const uint64_t chunk = 64ULL << 20;
	while (n) {
		size_t m = (size_t)(n < chunk ? n : chunk);
		if (fwrite(d, 1, m, f) != m)
			return -1;
		d += m;
		n -= m;
	}
	return 0;
}

static void zip_local_header(FILE *f, const char *name, uint32_t crc, uint32_t size,
			     uint16_t dtime, uint16_t ddate)
{
	put32(f, 0x04034b50);
	put16(f, 10);			/* version needed: stored */
	put16(f, 0);			/* flags */
	put16(f, 0);			/* method: stored */
	put16(f, dtime);
	put16(f, ddate);
	put32(f, crc);
	put32(f, size);
	put32(f, size);
	put16(f, (uint16_t)strlen(name));
	put16(f, 0);
	fwrite(name, 1, strlen(name), f);
}

static void zip_central_entry(FILE *f, const char *name, uint32_t crc, uint32_t size, uint32_t off,
			      uint16_t dtime, uint16_t ddate)
{
	put32(f, 0x02014b50);
	put16(f, 20);			/* made by */
	put16(f, 10);			/* needed */
	put16(f, 0);
	put16(f, 0);
	put16(f, dtime);
	put16(f, ddate);
	put32(f, crc);
	put32(f, size);
	put32(f, size);
	put16(f, (uint16_t)strlen(name));
	put16(f, 0);			/* extra */
	put16(f, 0);			/* comment */
	put16(f, 0);			/* disk */
	put16(f, 0);			/* internal attr */
	put32(f, 0);			/* external attr */
	put32(f, off);
	fwrite(name, 1, strlen(name), f);
}

static int sr_metadata(char *buf, size_t sz, uint64_t samplerate, int channels)
{
	int n = snprintf(buf, sz,
		"[global]\n"
		"sigrok version=0.5.2\n"
		"\n"
		"[device 1]\n"
		"capturefile=logic-1\n"
		"total probes=%d\n"
		"samplerate=%llu\n"
		"total analog=0\n", channels, (unsigned long long)samplerate);
	for (int i = 0; i < channels; i++)
		n += snprintf(buf + n, sz - n, "probe%d=D%d\n", i + 1, i);
	n += snprintf(buf + n, sz - n, "unitsize=1\n");
	return n;
}

/* sigrok session file: an uncompressed zip with version, metadata and data */
static int save_sr(FILE *f, const uint8_t *d, uint64_t n, uint64_t samplerate, int channels)
{
	static const char ver[] = "2";
	static const char *names[3] = { "version", "metadata", "logic-1-1" };
	char meta[1024];
	uint32_t len[3], crc[3], off[3];

	time_t now = time(NULL);
	struct tm *tm = localtime(&now);
	uint16_t dtime = (uint16_t)((tm->tm_hour << 11) | (tm->tm_min << 5) | (tm->tm_sec / 2));
	uint16_t ddate = (uint16_t)(((tm->tm_year - 80) << 9) | ((tm->tm_mon + 1) << 5) | tm->tm_mday);

	len[0] = 1;
	len[1] = (uint32_t)sr_metadata(meta, sizeof(meta), samplerate, channels);
	len[2] = (uint32_t)n;
	crc[0] = crc32_update(0, (const uint8_t *)ver, 1);
	crc[1] = crc32_update(0, (const uint8_t *)meta, len[1]);
	crc[2] = crc32_update(0, d, n);
	const uint8_t *content[3] = { (const uint8_t *)ver, (const uint8_t *)meta, d };

	uint32_t pos = 0;
	for (int i = 0; i < 3; i++) {
		off[i] = pos;
		zip_local_header(f, names[i], crc[i], len[i], dtime, ddate);
		if (write_all(f, content[i], len[i]))
			return -1;
		pos += 30 + (uint32_t)strlen(names[i]) + len[i];
	}
	uint32_t cd_off = pos;
	for (int i = 0; i < 3; i++) {
		zip_central_entry(f, names[i], crc[i], len[i], off[i], dtime, ddate);
		pos += 46 + (uint32_t)strlen(names[i]);
	}
	put32(f, 0x06054b50);
	put16(f, 0);
	put16(f, 0);
	put16(f, 3);
	put16(f, 3);
	put32(f, pos - cd_off);
	put32(f, cd_off);
	put16(f, 0);
	return 0;
}

static char *put_u64(char *p, uint64_t v)
{
	char tmp[21];
	int n = 0;
	do {
		tmp[n++] = (char)('0' + v % 10);
		v /= 10;
	} while (v);
	while (n)
		*p++ = tmp[--n];
	return p;
}

static int save_vcd(FILE *f, const uint8_t *d, uint64_t n, uint64_t samplerate, uint64_t trigger_pos,
		    int channels)
{
	enum { CHUNK = 256 * 1024, MAX_PER_SAMPLE = 22 + 3 * LA_CHANNELS };
	double ps = samplerate ? 1e12 / (double)samplerate : 1.0;
	char *ob = malloc((size_t)CHUNK * MAX_PER_SAMPLE + 256);
	if (!ob)
		return -1;

	time_t now = time(NULL);
	fprintf(f, "$date %s$end\n", ctime(&now));
	fprintf(f, "$version hsdaoh_la $end\n");
	fprintf(f, "$comment samplerate %llu Hz, trigger at sample %llu $end\n",
		(unsigned long long)samplerate, (unsigned long long)trigger_pos);
	fprintf(f, "$timescale 1 ps $end\n$scope module la $end\n");
	for (int i = 0; i < channels; i++)
		fprintf(f, "$var wire 1 %c D%d $end\n", '!' + i, i);
	fprintf(f, "$upscope $end\n$enddefinitions $end\n");

	uint8_t last = n ? d[0] : 0;
	for (uint64_t start = 0; start < n; start += CHUNK) {
		uint64_t end = start + CHUNK < n ? start + CHUNK : n;
		char *p = ob;
		uint64_t i = start;
		if (start == 0) {
			p += sprintf(p, "#0\n$dumpvars\n");
			for (int b = 0; b < channels; b++) {
				*p++ = (char)('0' + ((d[0] >> b) & 1));
				*p++ = (char)('!' + b);
				*p++ = '\n';
			}
			p += sprintf(p, "$end\n");
			i = 1;
		}
		for (; i < end; i++) {
			uint8_t c = d[i];
			if (c == last)
				continue;
			*p++ = '#';
			p = put_u64(p, (uint64_t)((double)i * ps + 0.5));
			*p++ = '\n';
			uint8_t x = c ^ last;
			for (int b = 0; b < channels; b++) {
				if (x & (1u << b)) {
					*p++ = (char)('0' + ((c >> b) & 1));
					*p++ = (char)('!' + b);
					*p++ = '\n';
				}
			}
			last = c;
		}
		if (fwrite(ob, 1, (size_t)(p - ob), f) != (size_t)(p - ob)) {
			free(ob);
			return -1;
		}
	}
	fprintf(f, "#%llu\n", (unsigned long long)((double)n * ps + 0.5));
	free(ob);
	return 0;
}

static double now_s(void)
{
	struct timespec ts;
	timespec_get(&ts, TIME_UTC);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static int save_capture(const char *path, const uint8_t *d, uint64_t n, uint64_t samplerate,
			uint64_t trigger_pos, int channels)
{
	int r;
	double t0 = now_s();
	FILE *f = fopen(path, "wb");
	if (!f) {
		fprintf(stderr, "Cannot create %s: %s\n", path, strerror(errno));
		return -1;
	}
	setvbuf(f, NULL, _IOFBF, 4 << 20);

	switch (out_type_from_name(path)) {
	case OUT_SR:  r = save_sr(f, d, n, samplerate, channels); break;
	case OUT_VCD: r = save_vcd(f, d, n, samplerate, trigger_pos, channels); break;
	default:      r = write_all(f, d, n); break;
	}
	if (fflush(f) || ferror(f))
		r = -1;
	if (fclose(f))
		r = -1;
	if (r) {
		fprintf(stderr, "Error writing %s: %s\n", path, strerror(errno));
		remove(path);
		return -1;
	}
	double dt = now_s() - t0;
	fprintf(stderr, "Written %s: %llu samples, trigger at sample %llu (%.1f s, %.0f MB/s)\n",
		path, (unsigned long long)n, (unsigned long long)trigger_pos, dt,
		dt > 0 ? n / dt / 1e6 : 0.0);
	return 0;
}

/* ------------------------------------------------------------------ capture */

enum { LA_ARMED, LA_CAPTURING, LA_DONE };
enum { CB_FREE, CB_FILLING, CB_SAVING };

#define MAX_CAPBUFS	2

typedef struct {
	uint8_t *data;
	uint64_t len;
	uint64_t trigger_pos;
	uint32_t srate;
	int channels;
	bool partial;
	int state;
	unsigned seq;
	char path[1200];
} capbuf_t;

typedef struct {
	trigger_t trig;
	uint64_t pre, post;
	int captures_wanted;		/* 0 = unlimited */
	int captures_done;		/* captures completed in RAM */
	char out_base[1024];
	uint64_t srate_override;
	bool ignore_gaps;
	bool quiet;

	volatile int state;
	uint64_t remaining;

	/* RAM capture buffers, written to disk by the saver thread */
	capbuf_t bufs[MAX_CAPBUFS];
	int nbufs;
	capbuf_t *cur;			/* reserved for the next/current capture */
	pthread_t saver;
	bool saver_running;
	pthread_mutex_t mtx;
	pthread_cond_t cond;
	bool saver_quit;
	unsigned save_seq;
	volatile int saves_done, saves_failed;
	bool told_busy;

	/* pre-trigger history ring */
	uint8_t *hist;
	uint64_t hist_len, hist_head;

	uint8_t prev;
	bool have_prev;

	/* gap detection */
	bool cnt_known, cnt_unsupported;
	uint64_t next_cnt;

	uint8_t *lin;
	size_t lin_cap;

	/* statistics (read by main thread) */
	volatile uint64_t samples_rx;
	volatile uint64_t gaps;
	volatile uint64_t lost_samples;
	uint32_t srate;
	int channels;			/* 8 (la8) or 4 (la4), 0 until known */
	bool error;
} la_t;

static void la_capture_name(la_t *la, char *out, size_t sz)
{
	if (la->captures_wanted == 1) {
		snprintf(out, sz, "%s", la->out_base);
		return;
	}
	const char *dot = strrchr(la->out_base, '.');
	const char *slash1 = strrchr(la->out_base, '/'), *slash2 = strrchr(la->out_base, '\\');
	if (dot && (slash1 > dot || slash2 > dot))
		dot = NULL;
	int stem = dot ? (int)(dot - la->out_base) : (int)strlen(la->out_base);
	snprintf(out, sz, "%.*s_%03d%s", stem, la->out_base, la->captures_done + 1, dot ? dot : "");
}

static void *saver_thread(void *arg)
{
	la_t *la = (la_t *)arg;

	pthread_mutex_lock(&la->mtx);
	while (1) {
		capbuf_t *b = NULL;
		for (int i = 0; i < la->nbufs; i++)
			if (la->bufs[i].state == CB_SAVING && (!b || la->bufs[i].seq < b->seq))
				b = &la->bufs[i];
		if (b) {
			pthread_mutex_unlock(&la->mtx);
#ifdef HSDAOH_LA_TEST_SAVE_DELAY_US
			usleep(HSDAOH_LA_TEST_SAVE_DELAY_US);	/* tests only: simulate a slow disk */
#endif
			int r = save_capture(b->path, b->data, b->len, b->srate, b->trigger_pos, b->channels);
			pthread_mutex_lock(&la->mtx);
			b->state = CB_FREE;
			b->len = 0;
			if (r)
				la->saves_failed++;
			else
				la->saves_done++;
			continue;
		}
		if (la->saver_quit)
			break;
		pthread_cond_wait(&la->cond, &la->mtx);
	}
	pthread_mutex_unlock(&la->mtx);
	return NULL;
}

/* Allocate (and touch, so the OS really provides it) the RAM buffers */
static int la_init(la_t *la)
{
	uint64_t cap = la->pre + la->post;

	la->nbufs = (la->captures_wanted == 1) ? 1 : MAX_CAPBUFS;
	if (cap > SIZE_MAX) {
		fprintf(stderr, "Capture too large for this system\n");
		return -1;
	}
	for (int i = 0; i < la->nbufs; i++) {
		la->bufs[i].data = malloc((size_t)cap);
		if (!la->bufs[i].data) {
			fprintf(stderr, "Not enough memory: need %d x %.0f MB for -p + -n. "
				"Reduce -n/-p%s.\n", la->nbufs, cap / 1e6,
				la->nbufs > 1 ? " or capture once (-c 1 needs half)" : "");
			return -1;
		}
		memset(la->bufs[i].data, 0, (size_t)cap);
		la->bufs[i].state = CB_FREE;
	}
	if (la->pre) {
		la->hist = malloc((size_t)la->pre);
		if (!la->hist) {
			fprintf(stderr, "Not enough memory for the pre-trigger buffer\n");
			return -1;
		}
	}

	pthread_mutex_init(&la->mtx, NULL);
	pthread_cond_init(&la->cond, NULL);
	if (pthread_create(&la->saver, NULL, saver_thread, la) != 0) {
		fprintf(stderr, "Cannot start saver thread\n");
		return -1;
	}
	la->saver_running = true;
	la->state = LA_ARMED;
	return 0;
}

static capbuf_t *la_reserve_buffer(la_t *la)
{
	if (la->cur)
		return la->cur;
	pthread_mutex_lock(&la->mtx);
	for (int i = 0; i < la->nbufs; i++) {
		if (la->bufs[i].state == CB_FREE) {
			la->bufs[i].state = CB_FILLING;
			la->bufs[i].len = 0;
			la->cur = &la->bufs[i];
			break;
		}
	}
	pthread_mutex_unlock(&la->mtx);
	return la->cur;
}

static void la_queue_save(la_t *la, bool partial)
{
	capbuf_t *b = la->cur;
	la->cur = NULL;
	b->partial = partial;
	la->captures_done++;
	if (!la->quiet || partial)
		fprintf(stderr, "\nCaptured %llu samples%s, writing %s in the background\n",
			(unsigned long long)b->len, partial ? " (stopped early)" : "", b->path);
	pthread_mutex_lock(&la->mtx);
	b->state = CB_SAVING;
	b->seq = la->save_seq++;
	pthread_cond_broadcast(&la->cond);
	pthread_mutex_unlock(&la->mtx);
}

/* Call after the stream has stopped: save a partial capture, wait for all
 * files to be written, free everything. */
static void la_finish(la_t *la, bool save_partial)
{
	if (la->state == LA_CAPTURING && la->cur && save_partial && la->cur->len)
		la_queue_save(la, true);
	if (la->cur) {
		pthread_mutex_lock(&la->mtx);
		la->cur->state = CB_FREE;
		pthread_mutex_unlock(&la->mtx);
		la->cur = NULL;
	}
	if (la->saver_running) {
		pthread_mutex_lock(&la->mtx);
		la->saver_quit = true;
		pthread_cond_broadcast(&la->cond);
		pthread_mutex_unlock(&la->mtx);
		pthread_join(la->saver, NULL);
		la->saver_running = false;
		pthread_cond_destroy(&la->cond);
		pthread_mutex_destroy(&la->mtx);
	}
	if (la->saves_failed)
		la->error = true;
	for (int i = 0; i < la->nbufs; i++) {
		free(la->bufs[i].data);
		la->bufs[i].data = NULL;
	}
	free(la->hist);
	free(la->lin);
	la->hist = NULL;
	la->lin = NULL;
}

static void hist_reset(la_t *la)
{
	la->hist_len = 0;
	la->hist_head = 0;
}

static void hist_push(la_t *la, const uint8_t *s, uint64_t n)
{
	uint64_t cap = la->pre;
	if (!cap || !n)
		return;
	if (n >= cap) {
		memcpy(la->hist, s + (n - cap), cap);
		la->hist_head = 0;
		la->hist_len = cap;
		return;
	}
	uint64_t first = cap - la->hist_head;
	if (first > n)
		first = n;
	memcpy(la->hist + la->hist_head, s, first);
	memcpy(la->hist, s + first, n - first);
	la->hist_head = (la->hist_head + n) % cap;
	la->hist_len = (la->hist_len + n > cap) ? cap : la->hist_len + n;
}

/* copy the newest n samples of the history, oldest first */
static void hist_copy(la_t *la, uint8_t *dst, uint64_t n)
{
	if (!n)
		return;
	uint64_t cap = la->pre;
	uint64_t start = (la->hist_head + cap - n) % cap;
	uint64_t first = cap - start;
	if (first > n)
		first = n;
	memcpy(dst, la->hist + start, first);
	memcpy(dst + first, la->hist, n - first);
}

static void la_on_gap(la_t *la, uint64_t lost)
{
	la->gaps++;
	la->lost_samples += lost;
	hist_reset(la);
	la->have_prev = false;

	if (la->state == LA_CAPTURING && !la->ignore_gaps) {
		fprintf(stderr, "\nData was lost during the capture (%llu samples): discarding it and re-arming.\n",
			(unsigned long long)lost);
		la->cur->len = 0;		/* keep the buffer reserved for the next try */
		la->state = LA_ARMED;
	} else if (la->state == LA_CAPTURING) {
		fprintf(stderr, "\nWarning: %llu samples lost inside the capture (continuing, -G given)\n",
			(unsigned long long)lost);
	} else if (!la->quiet) {
		fprintf(stderr, "\nData lost while armed (%llu samples), pre-trigger history restarted\n",
			(unsigned long long)lost);
	}
}

/* Scan for the trigger in s[from..n), return index or n */
static size_t la_find_trigger(la_t *la, const uint8_t *s, size_t from, size_t n)
{
	const uint8_t *tbl = la->trig.tbl;
	uint8_t p = la->have_prev ? la->prev : s[from];	/* no history: edges can't fire */
	for (size_t j = from; j < n; j++) {
		uint8_t c = s[j];
		if (tbl[((unsigned)p << 8) | c])
			return j;
		p = c;
	}
	return n;
}

/* Process n linear samples (already in time order) */
static void la_process_samples(la_t *la, const uint8_t *s, size_t n)
{
	size_t i = 0;

	while (i < n && la->state != LA_DONE) {
		if (la->state == LA_ARMED) {
			capbuf_t *b = la_reserve_buffer(la);
			if (!b) {
				/* all buffers still being written: keep history, don't trigger */
				if (!la->told_busy) {
					fprintf(stderr, "\nPrevious capture still being written to disk, not armed yet...\n");
					la->told_busy = true;
				}
				hist_push(la, s + i, n - i);
				break;
			}
			la->told_busy = false;

			size_t k = la_find_trigger(la, s, i, n);
			if (k == n) {
				hist_push(la, s + i, n - i);
				break;
			}

			/* trigger at s[k]: pre-trigger samples from history + this buffer */
			uint64_t from_chunk = k - i;
			uint64_t from_lin = from_chunk < la->pre ? from_chunk : la->pre;
			uint64_t from_hist = la->pre - from_lin;
			if (from_hist > la->hist_len)
				from_hist = la->hist_len;

			la_capture_name(la, b->path, sizeof(b->path));
			b->srate = la->srate_override ? (uint32_t)la->srate_override : la->srate;
			b->channels = la->channels ? la->channels : LA_CHANNELS;
			hist_copy(la, b->data, from_hist);
			memcpy(b->data + from_hist, s + k - from_lin, from_lin);
			b->len = from_hist + from_lin;
			b->trigger_pos = b->len;
			if (!la->quiet)
				fprintf(stderr, "\nTriggered (%s), capturing to memory...\n", la->trig.desc);

			la->remaining = la->post;
			la->state = LA_CAPTURING;
			i = k;
		}

		if (la->state == LA_CAPTURING) {
			capbuf_t *b = la->cur;
			uint64_t m = n - i;
			if (m > la->remaining)
				m = la->remaining;
			memcpy(b->data + b->len, s + i, m);
			b->len += m;
			la->remaining -= m;
			i += m;
			if (la->remaining == 0) {
				la_queue_save(la, false);
				hist_reset(la);
				if (la->captures_wanted && la->captures_done >= la->captures_wanted)
					la->state = LA_DONE;
				else
					la->state = LA_ARMED;
				/* the next trigger may fire on the sample right after this capture */
				la->prev = s[i - 1];
				la->have_prev = true;
			}
		}
	}

	if (n) {
		la->prev = s[n - 1];
		la->have_prev = true;
	}
}

/* channels in a stream of this format (libhsdaoh's data_info.format) */
static int la_channels_for_format(unsigned format)
{
	return format == HSDAOH_FORMAT_PIO_4BIT ? 4 : 8;
}

/* One buffer from the device: words = 16-bit link words, 8 channels: 2
 * samples each, first sample in the high byte; 4 channels: 4 samples each,
 * first sample in bits 15:12. data_cnt as reported by libhsdaoh. */
static void la_process_buffer(la_t *la, const uint8_t *buf, size_t len, uint64_t data_cnt, uint32_t srate,
			      int channels)
{
	size_t words = len / 2;
	size_t spw = channels == 4 ? 4 : 2;	/* samples per word */
	size_t n = words * spw;

	if (la->state == LA_DONE || !words)
		return;
	if (srate)
		la->srate = srate;
	if (la->channels != channels) {
		if (la->channels) {
			fprintf(stderr, "The device changed from %d to %d channels, stopping.\n",
				la->channels, channels);
			la->error = true;
			la->state = LA_DONE;
			return;
		}
		la->channels = channels;
		if (la->trig.used >> channels) {
			fprintf(stderr, "The trigger uses channels above D%d, but the device sends "
				"%d channels.\n", channels - 1, channels);
			la->error = true;
			la->state = LA_DONE;
			return;
		}
		if (!la->quiet)
			fprintf(stderr, "Device sends %d channels (D0..D%d).\n", channels, channels - 1);
	}

	/* gap detection via the device's word counter */
	if (!la->cnt_unsupported) {
		if (la->cnt_known && data_cnt == 0) {
			la->cnt_unsupported = true;
			fprintf(stderr, "Note: firmware does not report its sample counter; lost data cannot be detected.\n");
		} else if (la->cnt_known && data_cnt != la->next_cnt) {
			uint64_t lost = (data_cnt > la->next_cnt) ? (data_cnt - la->next_cnt) * spw : 0;
			la_on_gap(la, lost);
		}
		la->next_cnt = data_cnt + words;
		la->cnt_known = true;
	}

	if (la->lin_cap < n) {
		free(la->lin);
		la->lin = malloc(n);
		la->lin_cap = la->lin ? n : 0;
		if (!la->lin) {
			fprintf(stderr, "Out of memory\n");
			la->error = true;
			la->state = LA_DONE;
			return;
		}
	}
	/* undo the word packing (words arrive little endian) */
	if (spw == 2) {
		/* high byte is the earlier sample */
		for (size_t w = 0; w < words; w++) {
			la->lin[2 * w] = buf[2 * w + 1];
			la->lin[2 * w + 1] = buf[2 * w];
		}
	} else {
		/* bits 15:12 are the earliest sample, 3:0 the latest */
		for (size_t w = 0; w < words; w++) {
			uint8_t hi = buf[2 * w + 1], lo = buf[2 * w];
			la->lin[4 * w] = hi >> 4;
			la->lin[4 * w + 1] = hi & 0x0f;
			la->lin[4 * w + 2] = lo >> 4;
			la->lin[4 * w + 3] = lo & 0x0f;
		}
	}

	la_process_samples(la, la->lin, n);
	la->samples_rx += n;
}

/* ------------------------------------------------------------------ main */

#ifndef HSDAOH_LA_NO_MAIN

static hsdaoh_dev_t *dev = NULL;
static volatile int do_exit = 0;
static la_t la;

static void la_callback(hsdaoh_data_info_t *di)
{
	if (di->stream_id != 0 || do_exit)
		return;
	la_process_buffer(&la, di->buf, di->len, di->data_cnt, di->srate,
			  la_channels_for_format(di->format));
}

/* ------------------------------------------------------- Pico serial port */
/*
 * The la8 firmware takes commands on the Pico's USB serial port:
 *   "div <n>"  sample rate = system clock / n
 *   "?"        print the current setting
 * It answers with a status line ending in "commands: div <n>   ?", or with
 * "divider must be ..." if n is below the HDMI link limit.
 */

#ifdef _WIN32
typedef HANDLE la_ser_t;
#define LA_SER_INVALID INVALID_HANDLE_VALUE
#else
typedef int la_ser_t;
#define LA_SER_INVALID (-1)
#endif

static la_ser_t ser_open(const char *name)
{
#ifdef _WIN32
	char path[64];
	/* accept "COM5", "com5" or just "5"; \\.\ is needed for COM10 and up */
	if (isdigit((unsigned char)name[0]))
		snprintf(path, sizeof(path), "\\\\.\\COM%s", name);
	else if (!strncmp(name, "\\\\.\\", 4))
		snprintf(path, sizeof(path), "%s", name);
	else
		snprintf(path, sizeof(path), "\\\\.\\%s", name);

	HANDLE h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (h == INVALID_HANDLE_VALUE)
		return h;

	DCB dcb;
	memset(&dcb, 0, sizeof(dcb));
	dcb.DCBlength = sizeof(dcb);
	if (GetCommState(h, &dcb)) {
		dcb.BaudRate = 115200;		/* ignored by USB CDC, but must be valid */
		dcb.ByteSize = 8;
		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;
		dcb.fBinary = TRUE;
		dcb.fParity = FALSE;
		dcb.fOutxCtsFlow = FALSE;
		dcb.fOutxDsrFlow = FALSE;
		dcb.fDtrControl = DTR_CONTROL_ENABLE;	/* the Pico only sends while DTR is set */
		dcb.fRtsControl = RTS_CONTROL_ENABLE;
		dcb.fOutX = FALSE;
		dcb.fInX = FALSE;
		SetCommState(h, &dcb);
	}
	COMMTIMEOUTS to;
	memset(&to, 0, sizeof(to));
	to.ReadIntervalTimeout = MAXDWORD;	/* return what is there, wait up to 50 ms */
	to.ReadTotalTimeoutMultiplier = MAXDWORD;
	to.ReadTotalTimeoutConstant = 50;
	to.WriteTotalTimeoutConstant = 500;
	SetCommTimeouts(h, &to);
	EscapeCommFunction(h, SETDTR);
	return h;
#else
	int fd = open(name, O_RDWR | O_NOCTTY);
	if (fd < 0)
		return -1;
	struct termios tio;
	if (tcgetattr(fd, &tio) == 0) {
		cfmakeraw(&tio);
		cfsetispeed(&tio, B115200);
		cfsetospeed(&tio, B115200);
		tio.c_cflag |= CLOCAL | CREAD;
		tio.c_cc[VMIN] = 0;
		tio.c_cc[VTIME] = 1;		/* read() returns after 100 ms without data */
		tcsetattr(fd, TCSANOW, &tio);
	}
	int bits = TIOCM_DTR;
	ioctl(fd, TIOCMBIS, &bits);		/* the Pico only sends while DTR is set */
	return fd;
#endif
}

static void ser_close(la_ser_t s)
{
#ifdef _WIN32
	CloseHandle(s);
#else
	close(s);
#endif
}

static int ser_write(la_ser_t s, const char *str)
{
	size_t len = strlen(str);
#ifdef _WIN32
	DWORD done = 0;
	if (!WriteFile(s, str, (DWORD)len, &done, NULL) || done != len)
		return -1;
#else
	if (write(s, str, len) != (ssize_t)len)
		return -1;
#endif
	return 0;
}

/* read whatever arrives within ~50-100 ms; returns bytes read, <0 on error */
static int ser_read(la_ser_t s, char *buf, size_t size)
{
#ifdef _WIN32
	DWORD got = 0;
	if (!ReadFile(s, buf, (DWORD)size, &got, NULL))
		return -1;
	return (int)got;
#else
	ssize_t got = read(s, buf, size);
	return got < 0 ? -1 : (int)got;
#endif
}

/*
 * Send one command to the la8 firmware and print its answer.
 * Returns 0 if the firmware accepted it, -1 if it refused it or did not
 * answer, -2 if the serial port could not be used.
 */
static int pico_command(const char *port, const char *cmd)
{
	la_ser_t s = ser_open(port);
	if (s == LA_SER_INVALID) {
		fprintf(stderr, "Cannot open serial port %s (%s). Is it the Pico's port, and is no\n"
			"terminal program (PuTTY etc.) using it?\n", port,
#ifdef _WIN32
			"Windows error"
#else
			strerror(errno)
#endif
			);
		return -2;
	}

	/* drop anything the firmware printed earlier */
	char reply[2048];
	double t0 = now_s();
	while (now_s() - t0 < 0.2)
		if (ser_read(s, reply, sizeof(reply)) <= 0)
			break;

	char line[64];
	snprintf(line, sizeof(line), "%s\r", cmd);
	if (ser_write(s, line)) {
		fprintf(stderr, "Writing to serial port %s failed.\n", port);
		ser_close(s);
		return -2;
	}

	/* the status answer ends with the "commands:" line; errors are one line */
	size_t len = 0;
	bool done = false, ok = false;
	t0 = now_s();
	while (!done && now_s() - t0 < 2.0) {
		int n = ser_read(s, reply + len, sizeof(reply) - 1 - len);
		if (n < 0)
			break;
		len += (size_t)n;
		reply[len] = 0;
		if (strstr(reply, "commands:")) {
			ok = !strstr(reply, "divider must be") && !strstr(reply, "unknown command") &&
			     !strstr(reply, "channels must be");
			done = true;
		} else if (strstr(reply, "divider must be") && strchr(strstr(reply, "divider must be"), '\n')) {
			done = true;
		} else if (strstr(reply, "channels must be") && strchr(strstr(reply, "channels must be"), '\n')) {
			done = true;
		}
		if (len >= sizeof(reply) - 1)
			break;
	}
	ser_close(s);

	if (!len) {
		fprintf(stderr, "No answer from the Pico on %s (is la8, la4 or la48 running on it?).\n", port);
		return -1;
	}
	/* print the answer without the help line */
	for (char *l = strtok(reply, "\r\n"); l; l = strtok(NULL, "\r\n"))
		if (strncmp(l, "commands:", 9))
			fprintf(stderr, "Pico: %s\n", l);

	if (!done)
		fprintf(stderr, "Incomplete answer from the Pico.\n");
	return (done && ok) ? 0 : -1;
}

static uint64_t parse_count(const char *s)
{
	char *end;
	double v = strtod(s, &end);
	if (*end == 'k' || *end == 'K')
		v *= 1e3;
	else if (*end == 'M' || *end == 'm')
		v *= 1e6;
	else if (*end == 'G' || *end == 'g')
		v *= 1e9;
	return (uint64_t)(v + 0.5);
}

static void usage(void)
{
	fprintf(stderr,
		"hsdaoh_la, logic analyzer for the hsdaoh RP2350 'la8' (8 channels) and\n"
		"'la4' (4 channels) firmware; the channel count follows the device\n\n"
		"Usage: hsdaoh_la [options]\n"
		"\t-t trigger   comma separated channel:condition terms, all must be true\n"
		"\t             condition: 0 low, 1 high, r rising, f falling, c any edge\n"
		"\t             e.g. -t 0:r  or  -t 0:f,3:1  (default: trigger immediately)\n"
		"\t-n samples   samples to save from the trigger on (default 1M; k/M/G allowed)\n"
		"\t-p samples   samples to save before the trigger (default 0)\n"
		"\t-o file      output: .sr (PulseView/sigrok), .vcd, or raw .bin (default %s)\n"
		"\t-c count     number of captures, files get _001, _002... (default 1, 0 = endless)\n"
		"\t-r rate      override the sample rate in Hz written to the file\n"
		"\t-G           keep a capture even if data was lost during it\n"
		"\t-d index     device index (default 0)\n"
		"\t-s port      the Pico's USB serial port (Windows: COM5, Linux: /dev/ttyACM0)\n"
		"\t-D div       with -s: set the sample rate divider before capturing,\n"
		"\t             rate = Pico system clock / div (la8 140 MS/s and la4 280 MS/s\n"
		"\t             presets: 280 MHz / div)\n"
		"\t             -s without -D only shows the current setting\n"
		"\t-C 4|8       with -s, la48 firmware: switch to 4 channels (D0..D3,\n"
		"\t             up to 280 MS/s) or 8 channels (up to 140 MS/s)\n\n"
		"Captures are collected in RAM (1 byte per sample, -p + -n bytes; twice that\n"
		"with -c other than 1) and written to disk after the capture.\n", DEFAULT_OUTFILE);
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		do_exit = 1;
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	(void)signum;
	do_exit = 1;
}
#endif

int main(int argc, char **argv)
{
	int opt, r;
	int dev_index = 0;
	const char *trig = NULL;
	const char *ser_port = NULL;
	long set_div = 0, set_ch = 0;

	memset(&la, 0, sizeof(la));
	la.post = DEFAULT_POST;
	la.captures_wanted = 1;
	snprintf(la.out_base, sizeof(la.out_base), "%s", DEFAULT_OUTFILE);

	while ((opt = getopt(argc, argv, "t:n:p:o:c:r:Gd:s:D:C:qh")) != -1) {
		switch (opt) {
		case 't': trig = optarg; break;
		case 'n': la.post = parse_count(optarg); break;
		case 'p': la.pre = parse_count(optarg); break;
		case 'o': snprintf(la.out_base, sizeof(la.out_base), "%s", optarg); break;
		case 'c': la.captures_wanted = atoi(optarg); break;
		case 'r': la.srate_override = parse_count(optarg); break;
		case 'G': la.ignore_gaps = true; break;
		case 'd': dev_index = atoi(optarg); break;
		case 's': ser_port = optarg; break;
		case 'D': set_div = strtol(optarg, NULL, 10); break;
		case 'C': set_ch = strtol(optarg, NULL, 10); break;
		case 'q': la.quiet = true; break;
		default: usage();
		}
	}
	if (la.post == 0)
		la.post = 1;

	if ((set_div || set_ch) && !ser_port) {
		fprintf(stderr, "-D and -C need the Pico's serial port: -s COM5 (Windows) or -s /dev/ttyACM0\n");
		return 1;
	}
	if (set_ch && set_ch != 4 && set_ch != 8) {
		fprintf(stderr, "-C must be 4 or 8\n");
		return 1;
	}
	if (set_div < 0 || set_div > 65535) {
		fprintf(stderr, "-D must be 1..65535\n");
		return 1;
	}
	if (ser_port && set_ch) {
		/* la48 firmware: switch the channel count first (it may raise the
		 * divider to the new mode's minimum; -D below sets the rate) */
		char cmd[16];
		snprintf(cmd, sizeof(cmd), "ch %ld", set_ch);
		int pr = pico_command(ser_port, cmd);
		if (pr) {
			if (pr == -1)
				fprintf(stderr, "The Pico did not accept '%s' (-C needs the la48 firmware), "
					"not capturing.\n", cmd);
			return 1;
		}
	}
	if (ser_port && (set_div || !set_ch)) {
		char cmd[32];
		if (set_div)
			snprintf(cmd, sizeof(cmd), "div %ld", set_div);
		else
			snprintf(cmd, sizeof(cmd), "?");
		int pr = pico_command(ser_port, cmd);
		if (pr) {
			if (set_div && pr == -1)
				fprintf(stderr, "The Pico did not accept the divider, not capturing.\n");
			return 1;
		}
		/* the firmware reports the new rate in the HDMI metadata; give it a
		 * moment so the first frames we look at already carry it */
		usleep(200000);
	}
	if (out_type_from_name(la.out_base) == OUT_SR && la.pre + la.post > SR_MAX_DATA) {
		fprintf(stderr, "Capture too large for .sr (max %llu samples), use .bin\n",
			(unsigned long long)SR_MAX_DATA);
		return 1;
	}

	crc32_init();
	if (trigger_parse(&la.trig, trig))
		return 1;
	fprintf(stderr, "Allocating capture memory...\n");
	if (la_init(&la))
		return 1;

	r = hsdaoh_open(&dev, (uint32_t)dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open hsdaoh device #%d.\n", dev_index);
		la_finish(&la, false);
		return 1;
	}

#ifndef _WIN32
	struct sigaction sigact;
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
#else
	SetConsoleCtrlHandler((PHANDLER_ROUTINE)sighandler, TRUE);
#endif

	fprintf(stderr, "Trigger: %s, %llu samples before + %llu from trigger, %d capture(s) to %s\n"
		"Capture memory: %d x %.0f MB\n",
		la.trig.desc, (unsigned long long)la.pre, (unsigned long long)la.post,
		la.captures_wanted, la.out_base, la.nbufs, (la.pre + la.post) / 1e6);

	r = hsdaoh_start_stream(dev, la_callback, NULL, 0);

	uint64_t last_rx = 0;
	int ticks = 0;
	while (!do_exit && !r && la.state != LA_DONE) {
		usleep(100000);
		if (++ticks % 20 == 0 && !la.quiet) {
			uint64_t rx = la.samples_rx;
			const char *st = la.state == LA_CAPTURING ? "capturing" : "armed, waiting for trigger";
			fprintf(stderr, "\r%s: %.2f MS/s received, %llu gaps     ", st,
				(double)(rx - last_rx) / 2e6, (unsigned long long)la.gaps);
			last_rx = rx;
		}
	}

	hsdaoh_stop_stream(dev);	/* no more callbacks after this */

	if (do_exit)
		fprintf(stderr, "\nStopped by user.\n");
	else if (r)
		fprintf(stderr, "\nLibrary error %d.\n", r);

	fprintf(stderr, "Waiting for files to be written...\n");
	la_finish(&la, true);

	fprintf(stderr, "%d file(s) written%s, %llu samples received, %llu gaps (%llu samples lost).\n",
		la.saves_done, la.saves_failed ? " (some FAILED)" : "",
		(unsigned long long)la.samples_rx,
		(unsigned long long)la.gaps, (unsigned long long)la.lost_samples);

	hsdaoh_close(dev);
	return (r || la.error) ? 1 : 0;
}

#endif /* HSDAOH_LA_NO_MAIN */
