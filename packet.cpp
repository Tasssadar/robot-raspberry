#include <algorithm>
#include "packet.h"

Packet::Packet()
{
    cmd = 0;
    m_len = 0;
    m_witr = 0;
}

Packet::Packet(int cmd)
{
    cmd = cmd;
    m_len = 0;
    m_witr = 0;
}

Packet::~Packet()
{
}

void Packet::swap(Packet& other)
{
    data.swap(other.data);
    std::swap(cmd, other.cmd);
    std::swap(m_len, other.m_len);
    std::swap(m_witr, other.m_witr);
}

bool Packet::add(uint8_t b)
{
    switch(m_witr)
    {
        case 0:
        {
            if(b != 0x80)
                return false;
            break;
        }
        case 1:
        {
            m_len = b & 0xF;
            cmd = (b >> 4);
            data.reserve(m_len);
            break;
        }
        default:
        {
            if(m_witr >= m_len+2)
                return true;
            data.push_back(b);
            break;
        }
    }
    ++m_witr;
    return false;
}