#ifndef PACKET_H
#define PACKET_H

#include <vector>
#include <stdint.h>
#include <string>

class Packet
{
public:
    Packet();
    Packet(int cmd);
    ~Packet();

    bool add(uint8_t b);
    void swap(Packet& other);
    void clear();
    bool empty() const;
    void get_send_data(std::vector<char>& res) const;

    int cmd;
    std::vector<uint8_t> data;

    template <typename T> void write(T val);
    template <typename T> void read(T& val);
    void write(const std::string& val);
    void read(std::string& val);
    template <typename T> Packet& operator <<(T val);
    template <typename T> Packet& operator >>(T& val);
    Packet& operator <<(const std::string& val);
    Packet& operator >>(std::string& val);
    uint8_t operator [](size_t idx) const { return data[idx]; }
    uint8_t& operator [](size_t idx) { return data[idx]; }

private:
    int m_witr;
    int m_ritr;
    int m_len;
};

template <typename T>
void Packet::write(T val
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

template <typename T>
void Packet::read(T& val)
{
    uint8_t *r = ((uint8_t*)&val) + sizeof(T);
    for(int i = 0; i < sizeof(T); ++i)
    {
        --r;
        *r = data[m_ritr++];
    }
}

template <typename T>
Packet& Packet::operator >>(T& val)
{
    read<T>(val);
    return *this;
}

#endif