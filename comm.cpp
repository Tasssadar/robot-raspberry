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
#include "tcpserver.h"
#include "util.h"

#define PORT_DEV "/dev/ttyUSB0"
#define SPEED B115200

static void *run_write_thread(void *comm)
{
    ((Comm*)comm)->write_thread_work();
    return NULL;
}

Comm::Comm() 
{
    m_fd = -1;
    m_run_write_thread = false;
}

Comm::~Comm()
{
    this->destroy();
}

void Comm::initialize()
{
    LOGD("Starting");
    m_fd = open(PORT_DEV, O_RDWR | O_NOCTTY);
    if(m_fd < 0)
    {
        LOGE("Can't open: %s", strerror(errno));
        return;
    }

    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (m_fd, &tty) != 0)
    {
        LOGE("error \"%s\" from tcgetattr", strerror(errno));
        close(m_fd);
        m_fd = -1;
        return;
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
    tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    //tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (m_fd, TCSANOW, &tty) != 0)
    {
        LOGE("error %s from tcsetattr", strerror(errno));
        close(m_fd);
        m_fd = -1;
        return;
    }

    m_run_write_thread = true;
    pthread_create(&m_write_thread, 0, run_write_thread, this);
}

void Comm::destroy()
{
    if(m_run_write_thread)
    {
        m_run_write_thread = false;
        m_write_queue.cancelNotEmptyWait();
        pthread_join(m_write_thread, 0);
    }

    close(m_fd);
    m_fd = -1;
}

void Comm::send(char *str, int len)
{
    m_write_queue.push(std::vector<char>(str, str+len));
}

int Comm::available() const
{
    int bytes;
    if (::ioctl(m_fd, FIONREAD, &bytes) == -1) {
        LOGE("Failed to do FIONREAD ioctl");
        return -1;
    }
    return bytes;
}

void Comm::update(uint32_t diff)
{
    if(!isOpen())
        return;

    ssize_t av = available();

    char buff[1024];
    ssize_t chunk;
    ssize_t read = 0;
    while(read < av)
    {
        chunk = std::min((ssize_t)sizeof(buff), av);
        chunk = ::read(m_fd, buff, chunk);
        read += chunk;

        if(chunk > 0)
            sTunnelServer.write(buff, chunk);
        else
        {
            LOGE("Failed to read byte: %s", strerror(errno));
            break;
        }
    }
}

void Comm::write_thread_work()
{
    int res, sleep_counter = 0;
    Packet pkt;
    std::vector<char> data, send_data;

    while(m_run_write_thread)
    {
        if(!m_write_queue.waitForNotEmpty())
            continue;

        do
        {
            data = m_write_queue.pop();
            for(size_t i = 0; i < data.size(); ++i)
            {
                if(!pkt.add(data[i]))
                    continue;

                pkt.get_send_data(send_data);
                pkt.clear();

                res = ::write(m_fd, send_data.data(), send_data.size());
                if(res == -1)
                    LOGE("Failed to write bytes to comm: %s", strerror(errno));
                else if(res != (int)send_data.size())
                    LOGE("Failed to write %lu bytes to comm, %d written", send_data.size(), res);

                // If we write too fast, the port looses data. Sleeping after each
                // ~64B block seems to work pretty good
                sleep_counter += send_data.size();
                if(sleep_counter >= 64)
                {
                    usleep(15000);
                    sleep_counter = 0;
                }
            }
        }
        while(!m_write_queue.empty());
    }
}

