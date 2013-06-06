#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <vector>
#include <stdint.h>
#include <stack>

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

private:
    TcpServer();
    virtual ~TcpServer();

    void read_client(int fd, int av);
    void handle_cmds(char *buff, int len);

    int m_sock_fd;
    std::vector<int> m_clients;
};

#define sTcpServer TcpServer::instance()
#endif