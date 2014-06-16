#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <vector>
#include <stdint.h>
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
    TcpServer(uint16_t port);
    virtual ~TcpServer();

    void initialize();
    void destroy();

    int available(int fd) const;
    void update(uint32_t diff);
    void write(char *buff, int len);
    void write(char *str)
    {
        write(str, strlen(str));
    }
    void write(const char *fmt, ...);

protected:
    struct tcp_client
    {
        tcp_client(int fd)
        {
            this->fd = fd;
        }

        int fd;
        Packet pkt;
    };

    virtual void onClientAdded();
    virtual void onClientRm();

    virtual void read_client(tcp_client& cli) = 0;

    int m_sock_fd;
    uint16_t m_port;
    std::vector<tcp_client> m_clients;
};

class CommandTcpServer : public TcpServer
{
public:
    CommandTcpServer();

    static CommandTcpServer& instance()
    {
        static CommandTcpServer inst;
        return inst;
    }

    void write(const Packet &pkt);

private:
    void read_client(tcp_client& cli);
    void handle_packet(Packet& pkt);

    void onClientAdded();
    void onClientRm();
};

class TunnelTcpServer : public TcpServer
{
    TunnelTcpServer();

public:
    static TunnelTcpServer& instance()
    {
        static TunnelTcpServer inst;
        return inst;
    }

private:
    void read_client(tcp_client& cli);
};

#define sTcpServer CommandTcpServer::instance()
#define sTunnelServer TunnelTcpServer::instance()
#endif
