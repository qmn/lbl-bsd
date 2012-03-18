#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
/* POSIX */
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define LINE_LEN 38

extern void dcm_update(const char *);

static inline speed_t _serial_baud(const char *str) {
	static struct _assoc {
		char text[8];
		speed_t baud;
	} map[] = {
		{ .text = "1200", .baud = B1200 },
		{ .text = "2400", .baud = B2400 },
		{ .text = "4800", .baud = B4800 },
		{ .text = "9600", .baud = B9600 },
		{ .text = "19200", .baud = B19200 },
		{ .text = "38400", .baud = B38400 },
	};
	unsigned int i;
	for (i = (sizeof(map) / sizeof(struct _assoc)); i > 0;) {
		if (strcmp(str, map[--i].text) == 0) {
			return map[i].baud;
		}
	}
	return B0;
}

static int _serial_init(const char *dev, speed_t baud) {
	int fd;
	struct termios opt;
	char c;

	if ((fd = open(dev, O_RDONLY | O_NOCTTY)) < 0) {
		fputs("open: ", stderr);
		perror(dev);
		return -1;
	}
	/* Obtain current terminal parameters */
	if (tcgetattr(fd, &opt) != 0) {
		perror("tcgetattr");
		goto clean_fd;
	}
	/* Set baud rate */
	if (cfsetispeed(&opt, baud) != 0) {
		perror("cfsetispeed");
		goto clean_fd;
	}
	/* Enable receiver; select 8-bit character size,
	   no parity, one stop bit (8N1) */
	opt.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
	opt.c_cflag |= (CREAD | CS8 | CLOCAL);
	/* Select raw input */
	opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	/* Select single character reads, no timeout */
	opt.c_cc[VMIN]  = 1;
	opt.c_cc[VTIME] = 0;
	/* Set terminal parameters */
	if (tcsetattr(fd, TCSANOW, &opt) != 0) {
		perror("tcsetattr");
		goto clean_fd;
	}
	/* Synchronize */
	do {
		if (read(fd, &c, 1) <= 0) {
			perror("read");
			goto clean_fd;
		}  
	} while (c != '\n');
	do {
		if (read(fd, &c, 1) <= 0) {
			perror("read");
			goto clean_fd;
		}  
	} while (c != '\n');

	/* Select line length reads */
	opt.c_cc[VMIN] = LINE_LEN;	
	if (tcsetattr(fd, TCSANOW, &opt) != 0) {
		perror("tcsetattr");
		goto clean_fd;
	}
	return fd;
clean_fd:
	close(fd);
	return -1;
}

int main(int argc, char **argv) {
	extern char *optarg;
	extern int optopt;
	const char *dev;
	speed_t baud;
	int fd, c;

	dev = "/dev/ttyUSB0";
	baud = B38400;
	while ((c = getopt(argc, argv, ":d:b:")) != -1) {
		switch (c) {
			case 'd':
				dev = optarg;
				break;
			case 'b':
				if ((baud = _serial_baud(optarg)) == B0) {
					fprintf(stderr, "%s: invalid baud rate: %s\n",
					        argv[0], optarg);
					return 1;
				}
				break;
			default:
				fprintf(stderr, "%s: illegal option: -%c\n",
				        argv[0], optopt);
				return 1;
		}
	}
	if ((fd = _serial_init(dev, baud)) < 0) {
		return 1;
	}
	for (;;) {
		char buf[LINE_LEN];
		if (read(fd, buf, LINE_LEN) <= 0) {
			perror("read");
			return 1;
		}
		dcm_update(buf);
	}
	return 0;
}
