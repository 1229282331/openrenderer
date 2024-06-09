#include "buffer.hpp"


namespace openrenderer{
BufferType operator|(BufferType lhs, BufferType rhs)
{
    using underlying_t = std::underlying_type_t<BufferType>;  
    return static_cast<BufferType>(static_cast<underlying_t>(lhs) | static_cast<underlying_t>(rhs));  
}
    
Buffer::Buffer(const Buffer& rhs) : width(rhs.width), height(rhs.height) 
{
    size = rhs.size;
    pbyte = rhs.pbyte;
    format = rhs.format;
    buffer = new uint8_t[size];
    memcpy(buffer, rhs.buffer, sizeof(uint8_t)*size);
}
Buffer::Buffer(PixelFormat format_, int w, int h) : format(format_), width(w), height(h)
{
    switch (format)
    {
        case PixelFormat::RGB888:
            pbyte = 3;
            break;
        case PixelFormat::ARGB8888:
            pbyte = 4;
            break;
        case PixelFormat::GRAY8:
            pbyte = 3;
        default:
            pbyte = 3;
            break;
    }
    size = width * height * pbyte;
    buffer = new uint8_t[size];
    memset(buffer, 0, size);
}
Buffer::Buffer(PixelFormat format_, int w, int h, uint8_t* data) : format(format_), width(w), height(h)
{
    switch (format)
    {
        case PixelFormat::RGB888:
            pbyte = 3;
            break;
        case PixelFormat::ARGB8888:
            pbyte = 4;
            break;
        case PixelFormat::GRAY8:
            pbyte = 3;
        default:
            pbyte = 3;
            break;
    }
    size = width * height * pbyte;
    buffer = new uint8_t[size];
    memcpy(buffer, data, size);
}
Buffer& Buffer::operator=(const Buffer& rhs)
{
    delete []buffer;
    width = rhs.width;
    height = rhs.height;
    size = rhs.size;
    pbyte = rhs.pbyte;
    format = rhs.format;
    buffer = new uint8_t[size];
    memcpy(buffer, rhs.buffer, sizeof(uint8_t)*size);
    return *this;
}

void Buffer::depth2gray()
{
    union Tmp
    {
        float num;
        uint8_t arr[4];
    };
    
    for(int i=0; i<size; i+=4)
    {
        Tmp value;
        value.arr[0] = buffer[i];
        value.arr[1] = buffer[i+1];
        value.arr[2] = buffer[i+2];
        value.arr[3] = buffer[i+3];
        float gray_value = value.num; 
        buffer[i] = static_cast<uint8_t>(std::clamp(gray_value, 0.f, 1.f)*255.f);
        buffer[i+1] = static_cast<uint8_t>(std::clamp(gray_value, 0.f, 1.f)*255.f);
        buffer[i+2] = static_cast<uint8_t>(std::clamp(gray_value, 0.f, 1.f)*255.f);
        buffer[i+3] = 255;
    }
}

}