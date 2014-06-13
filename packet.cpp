#include <algorithm>
#include <stdio.h>
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
    printf("Packet:add: b: 0x%02X m_witr %d, cmd %d, m_len %d\n", b, m_witr, cmd, m_len);
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
            data.push_back(b);
            if(m_witr >= m_len+1)
                return true;
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

void Packet::read(std::string& val)
{
    val.clear();
    while(data[m_ritr] != 0 && m_ritr < 15)
        val.push_back(data[m_ritr++]);
    ++m_ritr; // skip \0
}

void Packet::write(const std::string& val)
{
    data.insert(data.end(), val.begin(), val.end());
    data.push_back(0);
}

Packet& Packet::operator <<(const std::string& val)
{
    write(val);
    return *this;
}

Packet& Packet::operator >>(std::string& val)
{
    read(val);
    return *this;
}
