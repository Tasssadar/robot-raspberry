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
#include <sys/socket.h>
#include <stdarg.h>
#include <ctype.h>

#include "tcpserver.h"
#include "camera.h"
#include "comm.h"
#include "util.h"

#define CMD_PORT 33000
#define TUNNEL_PORT 6511

TcpServer::TcpServer(uint16_t port)
{
    m_port = port;
}

TcpServer::~TcpServer()
{
    destroy();
}

void TcpServer::initialize()
{
    LOGD("starting on port %d", m_port);
    struct sockaddr_in serv_addr;
    m_sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(m_sock_fd < 0)
    {
        LOGE("failed to do socket() %s", strerror(errno));
        return;
    }

    bzero((char *)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(m_port);

    int optval = 1;
    setsockopt(m_sock_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

    if (bind(m_sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        LOGE("failed to bind %s\n", strerror(errno));
        close(m_sock_fd);
        return;
    }

    listen(m_sock_fd, 5);
    fcntl(m_sock_fd, F_SETFL, O_NDELAY); // non-blocking
}

void TcpServer::destroy()
{
    for(size_t i = 0; i < m_clients.size(); ++i)
        close(m_clients[i].fd);

    m_clients.clear();
    close(m_sock_fd);
}

int TcpServer::available(int fd) const
{
    int bytes;
    if (::ioctl(fd, FIONREAD, &bytes) == -1) {
        LOGE("failed to do FIONREAD ioctl on fd %d: %s", fd, strerror(errno));
        return -errno;
    }
    return bytes;
}

void TcpServer::write(char *buff, int len)
{
    int res;
    for(std::vector<tcp_client>::iterator itr = m_clients.begin(); itr != m_clients.end();++itr)
    {
        res = send((*itr).fd, buff, len, MSG_NOSIGNAL);
        if(res < 0)
            LOGE("failed on client %d: %s", (*itr).fd, strerror(errno));
    }
}

void TcpServer::write(const char *fmt, ...)
{
    char txt[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(txt, sizeof(txt), fmt, ap);
    va_end(ap);

    write(txt, strlen(txt));
}

void TcpServer::update(uint32_t diff)
{
    int res;
    while((res = accept(m_sock_fd, NULL, 0)) >= 0)
    {
        const int optval = 1;
        ioctl(res, FIONBIO, &optval);
        LOGD("Client connected: %d", res);
        m_clients.push_back(tcp_client(res));
        onClientAdded();
    }

    int av;
    for(std::vector<tcp_client>::iterator itr = m_clients.begin(); itr != m_clients.end(); )
    {
        av = available((*itr).fd);
        if(av > 0)
        {
            // read_client may execute commands and write(), so it may remove some clients.
            // keep it in mind
            read_client(*itr);
            ++itr;
        }
        else if(av < 0 || (av == 0 && recv((*itr).fd, NULL, 0, 0) == 0))
        {
            LOGD("Client disconnected: %d", (*itr).fd);
            ::close((*itr).fd);
            itr = m_clients.erase(itr);
            onClientRm();
        }
        else
            ++itr;
    }
}

void TcpServer::onClientAdded()
{

}

void TcpServer::onClientRm()
{

}



CommandTcpServer::CommandTcpServer() : TcpServer(CMD_PORT)
{
}

void CommandTcpServer::write(const Packet &pkt)
{
    std::vector<char> data;
    pkt.get_send_data(data);
    TcpServer::write(data.data(), data.size());
}

void CommandTcpServer::read_client(tcp_client& cli)
{
    char buff[64];

    ssize_t read;
    while(true)
    {
        read = ::read(cli.fd, buff, sizeof(buff));
        if(read <= 0)
            break;

        for(ssize_t i = 0; i < read; ++i)
        {
            if(cli.pkt.add(buff[i]))
            {
                handle_packet(cli.pkt);
                cli.pkt.clear();
            }
        }
    }
}

void CommandTcpServer::handle_packet(Packet& pkt)
{
    switch(pkt.cmd)
    {
        case CMSG_SET_VAR_INT:
        {
            std::string name;
            int32_t val;

            pkt >> name;
            pkt >> val;

            LOGD("CMSG_SET_VAR_INT %s = %d", name.c_str(), val);

            switch(name[0])
            {
                case 'c':
                    sCamera.setVar(name, val);
                    break;
            }
            break;
        }
        case CMSG_GET_VAR_INT:
        {
            std::string name;
            pkt >> name;

            int32_t ret = 0;
            switch(name[0])
            {
                case 'c':
                    ret = sCamera.getVar(name);
                    break;
            }

            LOGD("CMSG_GET_VAR_INT %s = %d", name.c_str(), ret);

            Packet res_pkt(SMSG_GET_VAR_INT);
            res_pkt << ret;
            write(res_pkt);
            break;
        }
        case CMSG_EXEC_ACT:
        {
            std::string name;
            pkt >> name;

            LOGD("CMSG_EXEC_ACT %s", name.c_str());

            switch(name[0])
            {
                case 'c':
                    sCamera.execAct(name);
                    break;
            }
            break;
        }
        default:
        {
            LOGE("Unknown opcode %d of len %d", pkt.cmd, pkt.data.size());
            break;
        }
    }
}

void CommandTcpServer::onClientAdded()
{
    if(m_clients.size() == 1)
        sCamera.open();
}

void CommandTcpServer::onClientRm()
{
    if(m_clients.empty())
        sCamera.close();
}

TunnelTcpServer::TunnelTcpServer() : TcpServer(TUNNEL_PORT)
{
}

void TunnelTcpServer::read_client(tcp_client& cli)
{
    char buff[64];

    ssize_t read;
    while(true)
    {
        read = ::read(cli.fd, buff, sizeof(buff));
        if(read <= 0)
            break;

        sComm.send(buff, read);
    }
}
