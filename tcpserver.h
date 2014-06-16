#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <vector>
#include <stdint.h>
#include <stack>
#include <string.h>

#include "packet.h"

enum
{
    CMSG_SET_VAR_INT = 10,
    CMSG_GET_VAR_INT = 11,
    SMSG_GET_VAR_INT = 12,
    CMSG_EXEC_ACT    = 13,
    SMSG_ACT_RES     = 14,
};

class TcpServer
{
public:
    static TcpServer& instance()
    {
        static TcpServer inst;
        return inst;
    }

    void initialize();
    void destroy();

    int available(int fd) const;
    void update(uint32_t diff);
    void write(char *buff, int len);
    void write(char *str)
    {
        write(str, strlen(str));
    }
    void write(const Packet &pkt);
    void write(const char *fmt, ...);

private:
    TcpServer();
    virtual ~TcpServer();

    struct tcp_client
    {
        tcp_client(int fd)
        {
            this->fd = fd;
        }

        int fd;
        Packet pkt;
    };

    void read_client(tcp_client& cli);
    void handle_packet(Packet& pkt);

    int m_sock_fd;
    std::vector<tcp_client> m_clients;
};

#define sTcpServer TcpServer::instance()
#endif