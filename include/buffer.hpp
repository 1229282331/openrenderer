#pragma once

#include "Eigen/Eigen"
#include <memory>
#include <vector>
#include <array>
#include <iostream>


namespace openrenderer{
enum class PixelFormat{ RGB888=1, ARGB8888, GRAY8 };
enum class ColorBit{ B=0, G, R, A };
enum class BufferType{ COLOR=0x01, DEPTH=0x02,  };
inline BufferType operator|(BufferType lhs, BufferType rhs)
{
    using underlying_t = std::underlying_type_t<BufferType>;  
    return static_cast<BufferType>(static_cast<underlying_t>(lhs) | static_cast<underlying_t>(rhs));  
}

template <typename T>
struct Buffer{
    T* buffer;
    int size;
    int pbyte;
    PixelFormat format;
    int width;
    int height;
    Buffer() : buffer(NULL), size(0), pbyte(3), format(PixelFormat::RGB888), width(0), height(0) {  }
    Buffer(const Buffer& rhs) : width(rhs.width), height(rhs.height) 
    {
        size = rhs.size;
        pbyte = rhs.pbyte;
        format = rhs.format;
        buffer = new T[size];
        memcpy(buffer, rhs.buffer, sizeof(T)*size);
    }
    Buffer(PixelFormat format_, int w, int h) : format(format_), width(w), height(h)
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
        buffer = new T[size];
        memset(buffer, 0, size);
    }
    Buffer(PixelFormat format_, int w, int h, T* data) : format(format_), width(w), height(h)
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
        buffer = new T[size];
        memcpy(buffer, data, size);
    }
    ~Buffer() { delete []buffer; }
    T& operator()(int i, int j, ColorBit bit) 
    { 
        i = std::clamp(i, 0, height-1);
        j = std::clamp(j, 0, width-1);
        return buffer[pbyte*(i*width+j)+int(bit)];  
    } 
    Buffer& operator=(const Buffer& rhs)
    {
        delete []buffer;
        width = rhs.width;
        height = rhs.height;
        size = rhs.size;
        pbyte = rhs.pbyte;
        format = rhs.format;
        buffer = new T[size];
        memcpy(buffer, rhs.buffer, sizeof(T)*size);
        return *this;
    }
    T  get(int i, int j, ColorBit bit) const 
    { 
        i = std::clamp(i, 0, height-1);
        j = std::clamp(j, 0, width-1);
        return buffer[pbyte*(i*width+j)+int(bit)];  
    } 
    void     set(int i, int j, ColorBit bit, T value) 
    { 
        i = std::clamp(i, 0, height-1);
        j = std::clamp(j, 0, width-1);
        buffer[pbyte*(i*width+j)+int(bit)]=value; 
    }
    void     clear() { memset(buffer, 0, size); }
};

inline void depth2gray(Buffer<uint8_t>& depthBuf)
{
    union Tmp
    {
        float num;
        uint8_t arr[4];
    };
    
    for(int i=0; i<depthBuf.size; i+=4)
    {
        Tmp value;
        value.arr[0] = depthBuf.buffer[i];
        value.arr[1] = depthBuf.buffer[i+1];
        value.arr[2] = depthBuf.buffer[i+2];
        value.arr[3] = depthBuf.buffer[i+3];
        float gray_value = value.num; 
        depthBuf.buffer[i] = static_cast<uint8_t>(std::clamp(gray_value, 0.f, 1.f)*255.f);
        depthBuf.buffer[i+1] = static_cast<uint8_t>(std::clamp(gray_value, 0.f, 1.f)*255.f);
        depthBuf.buffer[i+2] = static_cast<uint8_t>(std::clamp(gray_value, 0.f, 1.f)*255.f);
        depthBuf.buffer[i+3] = 255;
    }
}

}