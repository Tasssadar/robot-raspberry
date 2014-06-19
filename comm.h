#ifndef COMM_H
#define COMM_H

#include <string.h>
#include <stdint.h>
#include <vector>

#include "packet.h"
#include "util.h"

class Comm
{
public:
    static Comm& instance() 
    {
        static Comm inst;
        return inst;
    }

    void initialize();
    void destroy();

    int available() const;
    bool isOpen() const { return m_fd != -1; }
    void send(char *buff, int len);

    void update(uint32_t diff);

    void write_thread_work();

private:
    Comm();
    virtual ~Comm();

    int m_fd;
    volatile bool m_run_write_thread;
    pthread_t m_write_thread;
    SafeQueue<std::vector<char> > m_write_queue;
};

#define sComm Comm::instance()

#endif
