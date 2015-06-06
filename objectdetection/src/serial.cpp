#include "serial.h"

int Serial::classcount = 0;

Serial::Serial(char *portname, int baudrate, int len_read, int len_write,
		void *mutex) {
	int br = B0;
	switch (baudrate) {
	case 300:
		br = B300;
		break;
	case 600:
		br = B600;
		break;
	case 1200:
		br = B1200;
		break;
	case 2400:
		br = B2400;
		break;
	case 4800:
		br = B4800;
		break;
	case 9600:
		br = B9600;
		break;
	case 19200:
		br = B19200;
		break;
	case 38400:
		br = B38400;
		break;
	case 57600:
		br = B57600;
		break;
	case 115200:
		br = B115200;
		break;
	default:
		br = B9600;
		perror("SERIAL: invalid baudrate!\n");
	}

	this->newread = 0;

	// baudrate
	this->speed = br;

	// portname
	int plen = strlen(portname) + 1;
	this->portname = (char*) malloc(sizeof(char) * plen);
	memset(this->portname, '\0', plen);
	strcpy(this->portname, portname);

	// file handle
	this->fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (this->fd < 0) {
		printf("SERIAL %d: error %d opening %s: %s\n", m_id, errno, this->portname,
				strerror(errno));
        // TODO how can I shut down the program here? raise exception?
//		return NULL;
	} else {
		printf("SERIAL %d: opened port %s at %d\n", m_id, this->portname, this->fd);
	}

	set_interface_attribs(this->fd, this->speed, 0); // set speed, 8n1 (no parity)
	set_blocking(this->fd, 0);                // set no blocking

	max_read = len_read;
	max_write = len_write;
	buffer_read = (char*) malloc(max_read);
	buffer_write = (char*) malloc(max_write);
	resetRead();
	resetWrite();

	this->mutex = mutex;

	m_id = classcount;
	++classcount;

//	for (int i = 0; i < 100; ++i) {
//		write(fd, "OK", 3);
//		usleep(10000);
//	}

//	printf("SERIAL %d: Opened port %s with baudrate %d at fd=%d\n", m_id,
//			this->portname, baudrate, this->fd);
}

Serial::~Serial() {
	close(fd);
	free(portname);
	free(buffer_read);
	free(buffer_write);
}

int Serial::set_interface_attribs(int fd, int speed, int parity) {
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) {
		printf("SERIAL %d: error %d from tcgetattr\n", m_id, errno);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN] = 1;  //0          // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
									 // enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("SERIAL %d: error %d from tcsetattr\n", m_id, errno);
		return -1;
	}
	return 0;
}

void Serial::set_blocking(int fd, int should_block) {
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) {
		printf("SERIAL %d: error %d from tggetattr\n", m_id, errno);
		return;
	}

    if (should_block) {
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    } else {
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
    }
	if (tcsetattr(fd, TCSANOW, &tty) != 0)
		printf("SERIAL %d: error %d setting term attributes\n", m_id, errno);
}

//int Serial::sread(char *response, int maxlen) {
//	memset(response, '\0', maxlen);
//	buf = '\0';
//	int n = 0;
//	int spot = 0;
//	do {
//		n = read(fd, &buf, 1);
//		sprintf(&response[spot], "%c", buf);
//		if (n > 0) {
//			spot += n;
//			if (spot == maxlen) {
//				printf("SERIAL %d read: reached maxlen\n\r", m_id);
//				break;
//			}
//		}
//	} while (buf != CMD_ETX && n > 0);
//
//	if (spot > 0) {
//		printf("SERIAL %d received %d bytes: \n\r", m_id, spot);
//		for (int i = 0; i < spot; ++i) {
//			printf("%c", response[i]);
//		}
//		printf("\n\r");
//	}
//	return spot;
//}
//
//int Serial::swrite(char *tell, int len) {
////	write(this->fd, tell, len);
//	write(this->fd, "ab", 2);
//	usleep((7 + 25) * WRITE_WAIT);     // sleep enough to transmit the 7 plus
//	// receive 25:  approx 100 uS per char transmit
//
//	// check if received
//	buf = '\0';
//	int n = 0;
//	int i = 0;
//	for (i = 0; i < WRITE_MAX_TRY; ++i) {
//		n = read(fd, &buf, 1);
//		if (n > 0 && buf == CMD_ACK) {
//			break;
//		}
//		usleep(WRITE_WAIT);
//	}
////	printf("SERIAL %d waiting for check %d\n\r", m_id, i);
//	resetWrite();
//	return buf == CMD_ACK ? 1 : 0;
//}

int Serial::sread() {
//	resetRead();
	buf = CMD_NUL;
	int n = 0;
	int spot = 0;
	do {
		n = read(fd, &buf, 1);
		if (n > 0) {
				sprintf(&buffer_read[spot], "%c", buf);
				spot += n;
				if (spot == max_read) {
					printf("SERIAL %d read: reached maxlen\n\r", m_id);
//					spot = 0; // HACK find a better solution
					break;
				}
		}
//	} while (buf != CMD_ETX && n > 0);
	} while (n > 0);

	if (spot > 0) {
        printf("\n\r");
		printf("SERIAL %d received %d bytes: \n\r", m_id, spot);
		for (int i = 0; i < spot; ++i) {
			if (buffer_read[i] != '\0') {
				printf("%c", buffer_read[i]);
				fflush(stdout);
			}

		}
		printf("\n\r");
	}

	newread += spot;
    if (newread > max_read) newread = max_read;
	return spot;
}

int Serial::swrite() {
//	int len = strlen(buffer_write);
	int len = newwrite;
	if (len < 1)
		return 0;

//	printf("trying to write %s of length %d\n\r", buffer_write, len);
//	printf("trying to write %s\n\r", buffer_write);
	write(this->fd, buffer_write, len);

//	usleep((len + 25) * WRITE_WAIT);     // sleep enough to transmit the 7 plus
	resetWrite();
	return len;
}

void Serial::resetRead(char v) {
	memset(buffer_read, v, max_read * sizeof(char));
	newread = 0;
}

void Serial::resetWrite(char v) {
	memset(buffer_write, v, max_write * sizeof(char));
	newwrite = 0;
}

int Serial::newRead() {
//	return buffer_read[0] != CMD_NUL;
	return newread;
}

int Serial::newWrite() {
//	return buffer_write[0] != CMD_NUL;
	return newwrite;
}

bool Serial::isOpened() {
//	printf("fd: %d\n", this->fd);
	return this->fd - 1;
}

//bool Serial::check() {
//	return true;
//	// check if received
//	char check_buf = CMD_NUL;
//	int n = 0;
//	int i = 0;
//	for (i = 0; i < WRITE_MAX_TRY; ++i) {
//		n = read(fd, &check_buf, 1);
//		if (check_buf == CMD_ACK) {
//			break;
//		}
//		usleep(WRITE_WAIT);
//	}
//	printf("SERIAL %d waiting for check %d\n\r", m_id, i);
//	return check_buf == CMD_ACK;
//}

void *Serial::getMutex() {
	return mutex;
}

char *Serial::getRead() {
	return buffer_read;
}

void Serial::setWrite(char* val, int len) {
	resetWrite();
	memcpy(buffer_write, val, len);
	newwrite = len;
}

int Serial::getMaxRead() {
	return max_read;
}

int Serial::getMaxWrite() {
	return max_write;
}

int Serial::getID() {
	return m_id;
}

int Serial::getClassCount() {
	return classcount;
}
