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
BufferType operator|(BufferType lhs, BufferType rhs);

struct Buffer{
    uint8_t* buffer;
    int size;
    int pbyte;
    PixelFormat format;
    int width;
    int height;
    Buffer() : buffer(NULL), size(0), pbyte(3), format(PixelFormat::RGB888), width(0), height(0) {  }
    Buffer(const Buffer& rhs);
    Buffer(PixelFormat format_, int w, int h);
    Buffer(PixelFormat format_, int w, int h, uint8_t* data);
    ~Buffer() { delete []buffer; }
    uint8_t& operator()(int i, int j, ColorBit bit) 
    { 
        i = std::clamp(i, 0, height-1);
        j = std::clamp(j, 0, width-1);
        return buffer[pbyte*(i*width+j)+int(bit)];  
    } 
    Buffer& operator=(const Buffer& rhs);
    uint8_t  get(int i, int j, ColorBit bit) const 
    { 
        i = std::clamp(i, 0, height-1);
        j = std::clamp(j, 0, width-1);
        return buffer[pbyte*(i*width+j)+int(bit)];  
    } 
    void     set(int i, int j, ColorBit bit, uint8_t value) 
    { 
        i = std::clamp(i, 0, height-1);
        j = std::clamp(j, 0, width-1);
        buffer[pbyte*(i*width+j)+int(bit)]=value; 
    }
    void     clear() { memset(buffer, 0, size); }
    void     depth2gray();
};

}