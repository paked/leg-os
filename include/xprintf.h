// xprintf - a printf implementation which doesn't use any shared buffers, and
// as such is friendly for use with our write syscall style io.
//
// a single header library.

#include <stddef.h>
#include <stdarg.h>
#include <stdbool.h>

//
// header part of xprintf
//

int xprintf(uint32_t (*write)(char* buf, uint32_t len), char *fmt, ...);

#define XPRINTF_SIGNED (true)
#define XPRINTF_NOT_SIGNED (false)

#ifndef XPRINTF_BUFFER_SIZE
#define XPRINTF_BUFFER_SIZE 64
#endif

#ifdef XPRINTF_IMPLEMENTATION

//
// implementation of xprintf
//

struct xprintf_buffer {
	char buf[XPRINTF_BUFFER_SIZE];
	uint32_t len;
	uint32_t (*write)(char* buf, uint32_t len);
};

static void xprintf_buffer_flush(struct xprintf_buffer *buffer) {
	if (buffer->len == 0) {
		return;
	}

	if (buffer->write(buffer->buf, buffer->len) != buffer->len) {
		// FIXME(harrison): this is not good
		while (true);
	}

	buffer->len = 0;
}

static void xprintf_buffer_putc(struct xprintf_buffer *buffer, char c) {
	// always flush the buffer when it is filled up
	if (buffer->len + 1 >= XPRINTF_BUFFER_SIZE-1) {
		xprintf_buffer_flush(buffer);
	}

	buffer->buf[buffer->len] = c;
	buffer->len += 1;

	// flush after a newline character
	if (c == '\n') {
		xprintf_buffer_flush(buffer);
	}
}

static int xprintf_conv(struct xprintf_buffer *buffer, int32_t v, uint32_t radix, bool sigd) {
	char buf[XPRINTF_BUFFER_SIZE];
	uint32_t len = 0;
	bool neg = false;

	// we don't support weird radixes
	if (radix != 10 && radix != 8 && radix != 16) {
		return 0;
	}

	if (sigd && radix == 10 && v < 0) {
		neg = true;
		v = -v;
	}

	do {
		int n = v % radix;
		if (radix == 16 && n >= 10) {
			buf[len] = 'A' + n-10;
		} else {
			buf[len] = '0' + n;
		}

		len += 1;
		v /= radix;
	} while (v > 0 && len < XPRINTF_BUFFER_SIZE-1);

	if (neg) {
		xprintf_buffer_putc(buffer, '-');
	}

	if (radix == 16) {
		xprintf_buffer_putc(buffer, '0');
		xprintf_buffer_putc(buffer, 'x');
	}

	// put reversed string into output buffer
	for (int i = len - 1; i >= 0; i--) {
		xprintf_buffer_putc(buffer, buf[i]);
	}

	return len;
}

int xprintf(uint32_t (*write)(char* buf, uint32_t len), char *fmt, ...) {
	va_list args;
	struct xprintf_buffer buffer = {0};
	buffer.write = write;

	union {
		uint32_t u;
		int32_t  d;
		char* s;
	} u;

	int n = 0;

	va_start(args, fmt);
	for (; *fmt != 0; fmt += 1) {
		char next = *(fmt + 1);
		if (*fmt == '%' && next != 0) {
			// special case: %% renders as '%'
			if (next == '%') {
				fmt += 1;

				goto out;
			}

			// special case: %s prints a string
			if (next == 's') {
				u.s = va_arg(args, char*);

				for (; *u.s != 0; u.s += 1) {
					xprintf_buffer_putc(&buffer, *u.s);
					n += 1;
				}

				fmt += 1;

				continue;
			}

			if (next == 'u') {
				u.u = va_arg(args, uint32_t);

				n += xprintf_conv(&buffer, u.u, 10, XPRINTF_NOT_SIGNED);
			} else if (next == 'd') {
				u.d = va_arg(args, int32_t);

				n += xprintf_conv(&buffer, u.d, 10, XPRINTF_SIGNED);
			} else if (next == 'x') {
				u.u = va_arg(args, uint32_t);

				n += xprintf_conv(&buffer, u.u, 16, XPRINTF_NOT_SIGNED);
			}

			// skip type character
			fmt += 1;
			continue;
		}

out:

		xprintf_buffer_putc(&buffer, *fmt);
		n += 1;
	}
	va_end(args);

	xprintf_buffer_flush(&buffer);

	return n;
}

#endif
