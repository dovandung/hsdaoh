/*
 * hsdaoh - High Speed Data Acquisition over MS213x USB3 HDMI capture sticks
 *
 * hsdaoh_la: 8 channel logic analyzer front end.
 *
 * The device (RP2350 'la8' firmware) streams 8-bit samples continuously,
 * two samples per 16-bit link word (first sample in the high byte). This
 * program scans the stream for a trigger condition and saves the samples
 * around the trigger to a file (.sr for PulseView/sigrok, .vcd, or raw .bin).
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

#ifndef _WIN32
#include <unistd.h>
#include <getopt.h>
#define fseek64 fseeko
#else
#include <windows.h>
#include <getopt.h>
#define fseek64 _fseeki64
#ifndef usleep
#define usleep(t) Sleep((t)/1000)
#endif
#endif

#include "hsdaoh.h"

#define LA_CHANNELS		8
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
} trigger_t;

/*
 * Trigger spec: comma separated "channel:condition" terms, all must be true.
 *   channel   0..7 (D0..D7 = GP0..GP7 on the la8 firmware), 'D' prefix allowed
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
	return 0;
}

/* ------------------------------------------------------------------ writers */

enum { OUT_BIN, OUT_SR, OUT_VCD };

typedef struct {
	FILE *f;
	int type;
	char path[1200];
	uint64_t nsamples;
	uint64_t samplerate;
	uint64_t trigger_pos;
	/* .sr */
	uint32_t crc;
	int64_t data_hdr_off;
	uint32_t ver_crc, meta_crc, meta_len;
	int64_t meta_hdr_off;
	uint16_t dos_time, dos_date;
	/* .vcd: samples go to a raw temp file in real time, text is made on close */
	char tmp_path[1210];
	double ps_per_sample;
	char *vbuf;
} writer_t;

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

static void zip_local_header(writer_t *w, const char *name, uint32_t crc, uint32_t size)
{
	put32(w->f, 0x04034b50);
	put16(w->f, 10);		/* version needed: stored */
	put16(w->f, 0);			/* flags */
	put16(w->f, 0);			/* method: stored */
	put16(w->f, w->dos_time);
	put16(w->f, w->dos_date);
	put32(w->f, crc);
	put32(w->f, size);
	put32(w->f, size);
	put16(w->f, (uint16_t)strlen(name));
	put16(w->f, 0);
	fwrite(name, 1, strlen(name), w->f);
}

static void zip_central_entry(writer_t *w, const char *name, uint32_t crc, uint32_t size, uint32_t off)
{
	put32(w->f, 0x02014b50);
	put16(w->f, 20);		/* made by */
	put16(w->f, 10);		/* needed */
	put16(w->f, 0);
	put16(w->f, 0);
	put16(w->f, w->dos_time);
	put16(w->f, w->dos_date);
	put32(w->f, crc);
	put32(w->f, size);
	put32(w->f, size);
	put16(w->f, (uint16_t)strlen(name));
	put16(w->f, 0);			/* extra */
	put16(w->f, 0);			/* comment */
	put16(w->f, 0);			/* disk */
	put16(w->f, 0);			/* internal attr */
	put32(w->f, 0);			/* external attr */
	put32(w->f, off);
	fwrite(name, 1, strlen(name), w->f);
}

static const char sr_version[] = "2";

static int sr_metadata(char *buf, size_t sz, uint64_t samplerate)
{
	int n = snprintf(buf, sz,
		"[global]\n"
		"sigrok version=0.5.2\n"
		"\n"
		"[device 1]\n"
		"capturefile=logic-1\n"
		"total probes=%d\n"
		"samplerate=%llu\n"
		"total analog=0\n", LA_CHANNELS, (unsigned long long)samplerate);
	for (int i = 0; i < LA_CHANNELS; i++)
		n += snprintf(buf + n, sz - n, "probe%d=D%d\n", i + 1, i);
	n += snprintf(buf + n, sz - n, "unitsize=1\n");
	return n;
}

static int writer_open(writer_t *w, const char *path, uint64_t samplerate, uint64_t trigger_pos)
{
	memset(w, 0, sizeof(*w));
	snprintf(w->path, sizeof(w->path), "%s", path);
	w->type = out_type_from_name(path);
	w->samplerate = samplerate;
	w->trigger_pos = trigger_pos;
	snprintf(w->tmp_path, sizeof(w->tmp_path), "%s.tmp", path);
	const char *open_path = (w->type == OUT_VCD) ? w->tmp_path : path;
	w->f = fopen(open_path, "wb");
	if (!w->f) {
		fprintf(stderr, "Cannot create %s: %s\n", open_path, strerror(errno));
		return -1;
	}
	w->vbuf = malloc(4 << 20);
	if (w->vbuf)
		setvbuf(w->f, w->vbuf, _IOFBF, 4 << 20);

	if (w->type == OUT_SR) {
		time_t now = time(NULL);
		struct tm *tm = localtime(&now);
		w->dos_time = (uint16_t)((tm->tm_hour << 11) | (tm->tm_min << 5) | (tm->tm_sec / 2));
		w->dos_date = (uint16_t)(((tm->tm_year - 80) << 9) | ((tm->tm_mon + 1) << 5) | tm->tm_mday);

		char meta[1024];
		w->meta_len = (uint32_t)sr_metadata(meta, sizeof(meta), samplerate);
		w->ver_crc = crc32_update(0, (const uint8_t *)sr_version, 1);
		w->meta_crc = crc32_update(0, (const uint8_t *)meta, w->meta_len);

		zip_local_header(w, "version", w->ver_crc, 1);
		fwrite(sr_version, 1, 1, w->f);
		w->meta_hdr_off = ftell(w->f);
		zip_local_header(w, "metadata", w->meta_crc, w->meta_len);
		fwrite(meta, 1, w->meta_len, w->f);
		w->data_hdr_off = ftell(w->f);
		zip_local_header(w, "logic-1-1", 0, 0);	/* patched on close */
	} else if (w->type == OUT_VCD) {
		w->ps_per_sample = samplerate ? 1e12 / (double)samplerate : 1.0;
	}
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

/* Convert the raw temp file to VCD text (runs after the capture, not in real time) */
static int vcd_convert(writer_t *w)
{
	enum { IN_CHUNK = 256 * 1024, MAX_PER_SAMPLE = 22 + 3 * LA_CHANNELS };
	FILE *in = fopen(w->tmp_path, "rb");
	FILE *out = fopen(w->path, "wb");
	uint8_t *ib = malloc(IN_CHUNK);
	char *ob = malloc((size_t)IN_CHUNK * MAX_PER_SAMPLE + 64);
	int r = 0;

	if (!in || !out || !ib || !ob) {
		fprintf(stderr, "Cannot create %s: %s\n", w->path, strerror(errno));
		r = -1;
		goto done;
	}
	setvbuf(out, NULL, _IOFBF, 4 << 20);

	time_t now = time(NULL);
	fprintf(out, "$date %s$end\n", ctime(&now));
	fprintf(out, "$version hsdaoh_la $end\n");
	fprintf(out, "$comment samplerate %llu Hz, trigger at sample %llu $end\n",
		(unsigned long long)w->samplerate, (unsigned long long)w->trigger_pos);
	fprintf(out, "$timescale 1 ps $end\n$scope module la $end\n");
	for (int i = 0; i < LA_CHANNELS; i++)
		fprintf(out, "$var wire 1 %c D%d $end\n", '!' + i, i);
	fprintf(out, "$upscope $end\n$enddefinitions $end\n");

	uint64_t idx = 0;
	uint8_t last = 0;
	size_t n;
	while ((n = fread(ib, 1, IN_CHUNK, in)) > 0) {
		char *p = ob;
		size_t i = 0;
		if (idx == 0) {
			p += sprintf(p, "#0\n$dumpvars\n");
			for (int b = 0; b < LA_CHANNELS; b++) {
				*p++ = (char)('0' + ((ib[0] >> b) & 1));
				*p++ = (char)('!' + b);
				*p++ = '\n';
			}
			p += sprintf(p, "$end\n");
			last = ib[0];
			i = 1;
		}
		for (; i < n; i++) {
			uint8_t c = ib[i];
			if (c == last)
				continue;
			*p++ = '#';
			p = put_u64(p, (uint64_t)((double)(idx + i) * w->ps_per_sample + 0.5));
			*p++ = '\n';
			uint8_t d = c ^ last;
			for (int b = 0; b < LA_CHANNELS; b++) {
				if (d & (1u << b)) {
					*p++ = (char)('0' + ((c >> b) & 1));
					*p++ = (char)('!' + b);
					*p++ = '\n';
				}
			}
			last = c;
		}
		if (fwrite(ob, 1, (size_t)(p - ob), out) != (size_t)(p - ob)) {
			r = -1;
			break;
		}
		idx += n;
	}
	if (ferror(in))
		r = -1;
	fprintf(out, "#%llu\n", (unsigned long long)((double)idx * w->ps_per_sample + 0.5));
	if (fflush(out) || ferror(out))
		r = -1;
done:
	if (in)
		fclose(in);
	if (out && fclose(out))
		r = -1;
	free(ib);
	free(ob);
	remove(w->tmp_path);
	return r;
}

static int writer_write(writer_t *w, const uint8_t *s, size_t n)
{
	if (!n)
		return 0;

	if (w->type == OUT_SR)
		w->crc = crc32_update(w->crc, s, n);
	if (fwrite(s, 1, n, w->f) != n) {
		fprintf(stderr, "Write error on %s: %s\n", w->path, strerror(errno));
		return -1;
	}
	w->nsamples += n;
	return 0;
}

static int writer_close(writer_t *w)
{
	int r = 0;
	if (!w->f)
		return -1;

	if (w->type == OUT_SR) {
		uint32_t size = (uint32_t)w->nsamples;
		int64_t cd_off = (int64_t)w->data_hdr_off + 30 + 9 + (int64_t)w->nsamples;

		/* central directory */
		zip_central_entry(w, "version", w->ver_crc, 1, 0);
		zip_central_entry(w, "metadata", w->meta_crc, w->meta_len, (uint32_t)w->meta_hdr_off);
		zip_central_entry(w, "logic-1-1", w->crc, size, (uint32_t)w->data_hdr_off);
		int64_t cd_end = cd_off + 3 * 46 + 7 + 8 + 9;
		put32(w->f, 0x06054b50);
		put16(w->f, 0);
		put16(w->f, 0);
		put16(w->f, 3);
		put16(w->f, 3);
		put32(w->f, (uint32_t)(cd_end - cd_off));
		put32(w->f, (uint32_t)cd_off);
		put16(w->f, 0);

		/* patch CRC and sizes of the data entry */
		if (fseek64(w->f, w->data_hdr_off + 14, SEEK_SET) == 0) {
			put32(w->f, w->crc);
			put32(w->f, size);
			put32(w->f, size);
		} else {
			r = -1;
		}
	}

	if (fflush(w->f) || ferror(w->f))
		r = -1;
	if (fclose(w->f))
		r = -1;
	w->f = NULL;
	free(w->vbuf);
	w->vbuf = NULL;

	if (w->type == OUT_VCD) {
		if (r == 0)
			r = vcd_convert(w);
		else
			remove(w->tmp_path);
	}
	if (r)
		fprintf(stderr, "Error finishing %s\n", w->path);
	return r;
}

static void writer_abort(writer_t *w)
{
	if (w->f) {
		fclose(w->f);
		w->f = NULL;
		remove(w->type == OUT_VCD ? w->tmp_path : w->path);
	}
	free(w->vbuf);
	w->vbuf = NULL;
}

/* ------------------------------------------------------------------ capture */

enum { LA_ARMED, LA_CAPTURING, LA_DONE };

typedef struct {
	trigger_t trig;
	uint64_t pre, post;
	int captures_wanted;		/* 0 = unlimited */
	int captures_done;
	char out_base[1024];
	uint64_t srate_override;
	bool ignore_gaps;
	bool quiet;

	volatile int state;
	writer_t w;
	uint64_t remaining;
	uint64_t trig_abs;		/* absolute sample index of trigger */

	/* pre-trigger history ring */
	uint8_t *hist;
	uint64_t hist_len, hist_head;

	uint8_t prev;
	bool have_prev;

	/* gap detection */
	bool cnt_known, cnt_unsupported;
	uint64_t next_cnt;
	uint64_t abs_sample;		/* absolute index of next incoming sample */

	uint8_t *lin;
	size_t lin_cap;

	/* statistics (read by main thread) */
	volatile uint64_t samples_rx;
	volatile uint64_t gaps;
	volatile uint64_t lost_samples;
	uint32_t srate;
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

static int la_init(la_t *la)
{
	if (la->pre) {
		la->hist = malloc(la->pre);
		if (!la->hist) {
			fprintf(stderr, "Cannot allocate %llu bytes for pre-trigger buffer\n",
				(unsigned long long)la->pre);
			return -1;
		}
	}
	la->state = LA_ARMED;
	return 0;
}

static void la_free(la_t *la)
{
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

/* write the newest n samples of the history, oldest first */
static int hist_write(la_t *la, uint64_t n)
{
	if (!n)
		return 0;
	uint64_t cap = la->pre;
	uint64_t start = (la->hist_head + cap - n) % cap;
	uint64_t first = cap - start;
	if (first > n)
		first = n;
	if (writer_write(&la->w, la->hist + start, first))
		return -1;
	return writer_write(&la->w, la->hist, n - first);
}

static void la_on_gap(la_t *la, uint64_t lost)
{
	la->gaps++;
	la->lost_samples += lost;
	hist_reset(la);
	la->have_prev = false;

	if (la->state == LA_CAPTURING && !la->ignore_gaps) {
		fprintf(stderr, "\nData was lost during the capture (%llu samples): discarding %s and re-arming.\n",
			(unsigned long long)lost, la->w.path);
		writer_abort(&la->w);
		la->state = LA_ARMED;
	} else if (la->state == LA_CAPTURING) {
		fprintf(stderr, "\nWarning: %llu samples lost inside capture %s (continuing, -G given)\n",
			(unsigned long long)lost, la->w.path);
	} else if (!la->quiet) {
		fprintf(stderr, "\nData lost while armed (%llu samples), pre-trigger history restarted\n",
			(unsigned long long)lost);
	}
}

static void la_finish_capture(la_t *la, bool partial)
{
	uint64_t n = la->w.nsamples;
	int r = writer_close(&la->w);
	la->captures_done++;
	if (r)
		la->error = true;
	fprintf(stderr, "\n%s %s: %llu samples, trigger at sample %llu%s\n",
		r ? "FAILED" : "Saved", la->w.path, (unsigned long long)n,
		(unsigned long long)la->w.trigger_pos, partial ? " (stopped early)" : "");
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
			size_t k = la_find_trigger(la, s, i, n);
			if (k == n) {
				hist_push(la, s + i, n - i);
				break;
			}

			/* trigger at s[k] */
			uint64_t from_chunk = k - i;
			uint64_t want = la->pre;
			uint64_t from_lin = from_chunk < want ? from_chunk : want;
			uint64_t from_hist = want - from_lin;
			if (from_hist > la->hist_len)
				from_hist = la->hist_len;

			char name[1100];
			la_capture_name(la, name, sizeof(name));
			uint32_t srate = la->srate_override ? (uint32_t)la->srate_override : la->srate;
			if (writer_open(&la->w, name, srate, from_hist + from_lin)) {
				la->error = true;
				la->state = LA_DONE;
				return;
			}
			la->trig_abs = la->abs_sample + k;
			if (!la->quiet)
				fprintf(stderr, "\nTriggered (%s), capturing to %s ...\n", la->trig.desc, name);

			if (hist_write(la, from_hist) || writer_write(&la->w, s + k - from_lin, from_lin)) {
				writer_abort(&la->w);
				la->error = true;
				la->state = LA_DONE;
				return;
			}
			la->remaining = la->post;
			la->state = LA_CAPTURING;
			i = k;
		}

		if (la->state == LA_CAPTURING) {
			uint64_t m = n - i;
			if (m > la->remaining)
				m = la->remaining;
			if (writer_write(&la->w, s + i, m)) {
				writer_abort(&la->w);
				la->error = true;
				la->state = LA_DONE;
				return;
			}
			la->remaining -= m;
			i += m;
			if (la->remaining == 0) {
				la_finish_capture(la, false);
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

/* One buffer from the device: words = 16-bit link words, 2 samples each,
 * first sample in the high byte. data_cnt as reported by libhsdaoh. */
static void la_process_buffer(la_t *la, const uint8_t *buf, size_t len, uint64_t data_cnt, uint32_t srate)
{
	size_t words = len / 2;
	size_t n = words * 2;

	if (la->state == LA_DONE || !words)
		return;
	if (srate)
		la->srate = srate;

	/* gap detection via the device's word counter */
	if (!la->cnt_unsupported) {
		if (la->cnt_known && data_cnt == 0) {
			la->cnt_unsupported = true;
			fprintf(stderr, "Note: firmware does not report its sample counter; lost data cannot be detected.\n");
		} else if (la->cnt_known && data_cnt != la->next_cnt) {
			uint64_t lost = (data_cnt > la->next_cnt) ? (data_cnt - la->next_cnt) * 2 : 0;
			la->abs_sample += lost;
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
	/* undo the word packing: high byte is the earlier sample */
	for (size_t w = 0; w < words; w++) {
		la->lin[2 * w] = buf[2 * w + 1];
		la->lin[2 * w + 1] = buf[2 * w];
	}

	la_process_samples(la, la->lin, n);
	la->abs_sample += n;
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
	la_process_buffer(&la, di->buf, di->len, di->data_cnt, di->srate);
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
		"hsdaoh_la, 8 channel logic analyzer for the hsdaoh RP2350 'la8' firmware\n\n"
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
		"\t-d index     device index (default 0)\n", DEFAULT_OUTFILE);
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

	memset(&la, 0, sizeof(la));
	la.post = DEFAULT_POST;
	la.captures_wanted = 1;
	snprintf(la.out_base, sizeof(la.out_base), "%s", DEFAULT_OUTFILE);

	while ((opt = getopt(argc, argv, "t:n:p:o:c:r:Gd:qh")) != -1) {
		switch (opt) {
		case 't': trig = optarg; break;
		case 'n': la.post = parse_count(optarg); break;
		case 'p': la.pre = parse_count(optarg); break;
		case 'o': snprintf(la.out_base, sizeof(la.out_base), "%s", optarg); break;
		case 'c': la.captures_wanted = atoi(optarg); break;
		case 'r': la.srate_override = parse_count(optarg); break;
		case 'G': la.ignore_gaps = true; break;
		case 'd': dev_index = atoi(optarg); break;
		case 'q': la.quiet = true; break;
		default: usage();
		}
	}
	if (la.post == 0)
		la.post = 1;
	if (out_type_from_name(la.out_base) == OUT_SR && la.pre + la.post > SR_MAX_DATA) {
		fprintf(stderr, "Capture too large for .sr (max %llu samples), use .bin\n",
			(unsigned long long)SR_MAX_DATA);
		return 1;
	}

	crc32_init();
	if (trigger_parse(&la.trig, trig) || la_init(&la))
		return 1;

	r = hsdaoh_open(&dev, (uint32_t)dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open hsdaoh device #%d.\n", dev_index);
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

	fprintf(stderr, "Trigger: %s, saving %llu samples before + %llu from trigger, %d capture(s) to %s\n",
		la.trig.desc, (unsigned long long)la.pre, (unsigned long long)la.post,
		la.captures_wanted, la.out_base);

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

	if (la.state == LA_CAPTURING)
		la_finish_capture(&la, true);
	if (do_exit)
		fprintf(stderr, "\nStopped by user.\n");
	else if (r)
		fprintf(stderr, "\nLibrary error %d.\n", r);

	fprintf(stderr, "%d capture(s) saved, %llu samples received, %llu gaps (%llu samples lost).\n",
		la.captures_done, (unsigned long long)la.samples_rx,
		(unsigned long long)la.gaps, (unsigned long long)la.lost_samples);

	hsdaoh_close(dev);
	la_free(&la);
	return (r || la.error) ? 1 : 0;
}

#endif /* HSDAOH_LA_NO_MAIN */
