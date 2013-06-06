#ifndef PACKET_H
#define PACKET_H

#include <vector>
#include <stdint.h>

class Packet
{
public:
    Packet();
    Packet(int cmd);
    ~Packet();

    bool add(uint8_t b);
    void swap(Packet& other);

    int cmd;
    std::vector<uint8_t> data;

    template <typename T> void write(T val);
    template <typename T> Packet& operator <<(T val);

private:
    int m_witr;
    int m_len;
};

template <typename T>
void Packet::write(T val)
{
    for(int i = 1; i <= sizeof(T); ++i)
        data.push_back((val >> ((sizeof(T)-i)*8)) & 0xFF);
}

template <typename T>
Packet& Packet::operator <<(T val)
{
    write<T>(val);
    return *this;
}

#endif