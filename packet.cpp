#include <algorithm>
#include "packet.h"

Packet::Packet()
{
    clear();
}

Packet::Packet(int cmd)
{
    clear();
    cmd = cmd;
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

void Packet::clear()
{
    cmd = 0;
    m_len = 0;
    m_witr = 0;
    m_ritr = 0;
    data.clear();
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

void Packet::get_send_data(std::vector<char>& res) const
{
    res.clear();
    res.push_back(0x80);
    res.push_back((data.size() << 4) | cmd);
    res.insert(res.end(), data.begin(), data.end());

    if(res.size() > 17)
        res.resize(17);
}
