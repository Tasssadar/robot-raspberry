#include <string.h>
#include <errno.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>

#include "comm.h"

#define PORT_DEV "/dev/ttyUSB0"
#define SPEED B115200

Comm::Comm() 
{
    m_fd = -1;
}

Comm::~Comm()
{
    this->destroy();
}

void Comm::initialize()
{
    printf("Comm: Starting\n");
    m_fd = open(PORT_DEV, O_RDWR | O_NOCTTY | O_NDELAY);
    if(m_fd < 0)
    {
        fprintf(stderr, "Can't open the serial port: %s\n", strerror(errno));
        throw "Can't open serial port";
    }


    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (m_fd, &tty) != 0)
    {
        fprintf(stderr, "error %d from tcgetattr", errno);
        throw "Can't open serial port";
    }

    cfsetospeed (&tty, SPEED);
    cfsetispeed (&tty, SPEED);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // ignore break signal
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    //tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (m_fd, TCSANOW, &tty) != 0)
    {
        fprintf(stderr, "error %d from tcsetattr", errno);
        throw "Can't open serial port";
    }
}

void Comm::destroy()
{
    close(m_fd);
}

void Comm::send(char *str, int len)
{
    int res = write(m_fd, str, len);
    if(res != len)
    {
        if(res == -1)
            fprintf(stderr, "Failed to write bytes to comm: %s\n", strerror(errno));
        else
            fprintf(stderr, "Failed to write %d bytes to comm, %d written\n", len, res);
    }
}

void Comm::send(char c)
{
    send(&c, 1);
}

void Comm::send(const Packet& pkt)
{
    char buff[2];
    buff[0] = 0x80;
    buff[1] = (pkt.cmd << 4) | (pkt.data.size() & 0xF);

    int res = write(m_fd, buff, sizeof(buff));
    if(res == sizeof(buff))
        res = write(m_fd, pkt.data.data(), pkt.data.size());

    if(res == -1)
        fprintf(stderr, "Failed to write bytes to comm: %s\n", strerror(errno));
}

int Comm::available() const
{
    int bytes;
    if (::ioctl(m_fd, FIONREAD, &bytes) == -1) {
        fprintf(stderr, "Failed to do FIONREAD ioctl\n");
        return -1;
    }
    return bytes;
}

void Comm::update(uint32_t diff)
{
    int av = available();
    uint8_t b;
    int res;
    for(int i = 0; i < av; ++i)
    {
        res = read(m_fd, &b, 1);
        if(res == 1)
        {
            if(m_pkt.add(b))
            {
                // handle packet
            }
        }
        else
            fprintf(stderr, "Failed to read byte: %s\n", strerror(errno));
    }
}

