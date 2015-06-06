//http://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
//http://stackoverflow.com/questions/18108932/linux-c-serial-port-reading-writing
#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>             /* standard I/O routines                      */
//#include <pthread.h>           /* pthread functions and data structures      */
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include "fcntl.h"

// locals
#include "defines.h"

class Serial {
private:
    static int classcount;
	char *portname;
	int speed;
	int fd;
	char *buffer_read;
	char *buffer_write;
	char buf;
	int max_read;
	int max_write;
    int m_id;
    int newread;
    int newwrite;

	int set_interface_attribs(int fd, int speed, int parity);
	void set_blocking(int fd, int should_block);
	void* mutex;

public:
	Serial(char *portname, int baudrate, int len_read, int len_write, void *mutex);
	~Serial();

//	int sread(char *response, int maxlen);
//	int swrite(char *tell, int len);

	int sread();
	int swrite();

	void resetRead(char v = CMD_NUL);
	void resetWrite(char v = CMD_NUL);

	int newRead();
	int newWrite();
	bool isOpened();
//    bool check();

	void* getMutex();
	char* getRead();
	void setWrite(char* val, int len);
	int getMaxRead();
	int getMaxWrite();
    int getID();
    static int getClassCount();
};
#endif // SERIAL_H
