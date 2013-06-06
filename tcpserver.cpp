#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <string.h>
#include <errno.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>

#include "tcpserver.h"

#define PORT 33000

TcpServer::TcpServer()
{
    
}

TcpServer::~TcpServer()
{
    destroy();
}

void TcpServer::initialize()
{
    printf("TcpServer: Starting\n");
    struct sockaddr_in serv_addr;
    m_sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(m_sock_fd < 0)
    {
        fprintf(stderr, "TcpServer: Failed to do socket() %s\n", strerror(errno));
        return;
    }

    bzero((char *)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(PORT);

    int optval = 1;
    setsockopt(m_sock_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);
    if (bind(m_sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        fprintf(stderr, "TcpServer: Failed to bind %s\n", strerror(errno));
        close(m_sock_fd);
        return;
    }

    listen(m_sock_fd, 5);
    fcntl(m_sock_fd, F_SETFL, O_NDELAY); // non-blocking
}

void TcpServer::destroy()
{
    for(size_t i = 0; i < m_clients.size(); ++i)
        close(m_clients[i]);

    close(m_sock_fd);
}

int TcpServer::available(int fd) const
{
    int bytes;
    if (::ioctl(fd, FIONREAD, &bytes) == -1) {
        fprintf(stderr, "TcpServer:: Failed to do FIONREAD ioctl\n");
        return -errno;
    }
    return bytes;
}

void TcpServer::write(char *buff, int len)
{
    int res;
    for(std::vector<int>::iterator itr = m_clients.begin(); itr != m_clients.end(); ++itr)
    {
        res = ::write(*itr, buff, len);

        if(res < 0)
        {
            ::close(*itr);
            itr = m_clients.erase(itr);
        }
    }
}

void TcpServer::read_client(int fd, int av)
{
    char buff[16];

    int chunk;
    int read = 0;
    while(read != av)
    {
        chunk = std::min(sizeof(buff), (size_t)av);
        ::read(fd, buff, chunk);
        read += chunk;
    }

    handle_cmds(buff, av);
}

void TcpServer::handle_cmds(char *buff, int len)
{
    for(int i = 0; i < len; ++i)
    {
        switch(buff[i])
        {
            default:
                printf("%c", buff[i]);
                fflush(stdout);
                break;
        }
    }
}

void TcpServer::update(uint32_t diff)
{
    int res;
    while((res = accept(m_sock_fd, NULL, 0)) >= 0)
        m_clients.push_back(res);

    int av;
    for(std::vector<int>::iterator itr = m_clients.begin(); itr != m_clients.end(); ++itr)
    {
        av = available(*itr);
        if(av == 0)
            continue;

        if(av > 0)
            read_client(*itr, av);
        else
        {
            ::close(*itr);
            itr = m_clients.erase(itr);
        }
    }
}