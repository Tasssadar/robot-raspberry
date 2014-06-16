#ifndef COMM_H
#define COMM_H

#include <string.h>
#include <stack>
#include <stdint.h>
#include "packet.h"

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
    void send(char c);
    void send(const Packet& pkt);

    Comm& operator <<(const std::string& str)
    {
        send((char*)str.c_str(), str.size());
        return *this;
    }

    Comm& operator <<(const char *str)
    {
        send((char*)str, strlen(str));
        return *this;
    }

    void update(uint32_t diff);

private:
    Comm();
    virtual ~Comm();

    int m_fd;
    Packet m_pkt;
};

#define sComm Comm::instance()

#endif
