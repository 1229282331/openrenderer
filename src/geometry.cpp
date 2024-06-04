#pragma once

#include "geometry.hpp"

#define MY_PI 3.1415926

extern openrenderer::Uniform ubo;

namespace openrenderer{
BufferType operator|(BufferType lhs, BufferType rhs)
{
    using underlying_t = std::underlying_type_t<BufferType>;  
    return static_cast<BufferType>(static_cast<underlying_t>(lhs) | static_cast<underlying_t>(rhs));  
}

Eigen::Matrix4f scale(float rateX, float rateY, float rateZ)
{
    Eigen::Matrix4f Scale = Eigen::Matrix4f::Identity();
    Scale(0, 0) *= rateX;
    Scale(1, 1) *= rateY;
    Scale(2, 2) *= rateZ;
    return Scale;
}

Eigen::Matrix4f rotate(float angle, const Eigen::Vector3f& v)
{
    float const a = angle;
    float const c = cos(a);
    float const s = sin(a);

    Eigen::Vector3f axis(v.normalized());
    Eigen::Vector3f temp((1.0 - c) * axis);

    Eigen::Matrix4f Rotate;
    Rotate(0, 0) = c + temp(0) * axis[0];
    Rotate(1, 0) = temp[0] * axis[1] + s * axis[2];
    Rotate(2, 0) = temp[0] * axis[2] - s * axis[1];

    Rotate(0, 1) = temp[1] * axis[0] - s * axis[2];
    Rotate(1, 1) = c + temp[1] * axis[1];
    Rotate(2, 1) = temp[1] * axis[2] + s * axis[0];

    Rotate(0, 2) = temp[2] * axis[0] + s * axis[1];
    Rotate(1, 2) = temp[2] * axis[1] - s * axis[0];
    Rotate(2, 2) = c + temp[2] * axis[2];

    Rotate(0, 3) = 0.f;
    Rotate(1, 3) = 0.f;
    Rotate(2, 3) = 0.f;
    Rotate.row(3) = Eigen::Vector4f(0.f, 0.f, 0.f, 1.f);
    
    return Rotate;
}

Eigen::Matrix4f translate(const Eigen::Vector3f& v)
{
    Eigen::Matrix4f Translate = Eigen::Matrix4f::Identity();
    Translate(0, 3) = v[0];
    Translate(1, 3) = v[1];
    Translate(2, 3) = v[2];
    Translate(3, 3) = 1.f;

    return Translate;
}

Eigen::Matrix4f lookAt(const Eigen::Vector3f& eyePos, const Eigen::Vector3f& center, const Eigen::Vector3f& up)
{
    Eigen::Vector3f const f((center - eyePos).normalized());
    Eigen::Vector3f const s((f.cross(up)).normalized());
    Eigen::Vector3f const u((s.cross(f)));

    Eigen::Matrix4f Result = Eigen::Matrix4f::Identity();
    Result(0, 0) = s[0];
    Result(1, 0) = s[1];
    Result(2, 0) = s[2];
    Result(0, 1) = u[0];
    Result(1, 1) = u[1];
    Result(2, 1) = u[2];
    Result(0, 2) =-f[0];
    Result(1, 2) =-f[1];
    Result(2, 2) =-f[2];
    Result(3, 0) =-s.dot(eyePos);
    Result(3, 1) =-u.dot(eyePos);
    Result(3, 2) = f.dot(eyePos);
    return Result.transpose();
}

Eigen::Matrix4f perspective(float fovy, float aspect, float z_near, float z_far)
{
    // assert(abs(aspect - std::numeric_limits<float>::epsilon()) > 0.f);

    float const tanHalfFovy = tan(fovy / 2.f);

    Eigen::Matrix4f Result = Eigen::Matrix4f::Identity();
    Result(0, 0) = 1.f / (aspect * tanHalfFovy);
    Result(1, 1) = 1.f / (tanHalfFovy);
    Result(2, 2) = (z_near + z_far) / -(z_far - z_near);
    Result(3, 2) = - 1.f;
    Result(2, 3) = -(2 * z_far * z_near) / (z_far - z_near);
    return Result;
}

Eigen::Vector3f inverse2Dto3D(int x, int y, float z, int w, int h, const Eigen::Matrix4f& model, const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection)
{
    Eigen::Vector4f worldPos;
    //inverse viewport transform
    worldPos.x() = 2 * x / float(w) - 1.f;
    worldPos.y() = 2 * y / float(h) - 1.f;
    worldPos.z() = z;
    worldPos.w() = 1.f;
    //inverse MVP
    worldPos = model.inverse() * view.inverse() * projection.inverse() * worldPos;
    worldPos /= worldPos.w();

    return Eigen::Vector3f(worldPos.x(), worldPos.y(), worldPos.z());
}

void RT_decompose(const Eigen::Matrix4f& RT, Eigen::Matrix4f& R, Eigen::Matrix4f& T)
{
    R = RT;
    R.col(3) = Eigen::Vector4f(0.f, 0.f, 0.f, 1.f);
    T = Eigen::Matrix4f::Identity();
    T.col(3) = RT.col(3);
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




Framebuffers::Framebuffers(int w, int h, bool enable_color, bool enable_depth, PixelFormat color_format, PixelFormat depth_format) : width(w), height(h)
{
    if(enable_color)
        color_buffer = std::make_unique<Buffer>(color_format, w, h);
    if(enable_depth)
    {
        int shadowMap_num = 0;
        for(int i=0; i<ubo.lights.size(); i++)
        {
            if(ubo.lights[i].hasShadowMap)
            {
                depth_buffer.push_back(std::make_unique<Buffer>(depth_format, w, h));
                memset(depth_buffer[shadowMap_num]->buffer, 255, depth_buffer[shadowMap_num]->size);
                shadowMap_num++;
            }
        }
    }
    z_buffer = new float[width*height];
    std::fill(z_buffer, z_buffer+width*height, std::numeric_limits<float>::max());
}
Framebuffers::~Framebuffers()
{
    delete []z_buffer;
    color_buffer.reset();
    for(int i=0; i<depth_buffer.size(); i++)
        depth_buffer[i].reset();
}
Framebuffers::Framebuffers(const Framebuffers& rhs)
{
    width = rhs.width;
    height = rhs.height;
    if(color_buffer.get())
        color_buffer.reset();
    color_buffer = std::make_unique<Buffer>(*rhs.color_buffer);
    if(depth_buffer.size())
    {
        for(int i=0; i<depth_buffer.size(); i++)
            depth_buffer[i].reset();
        depth_buffer.clear();
        for(int i=0; i<rhs.depth_buffer.size(); i++)
            depth_buffer.push_back(std::make_unique<Buffer>(*rhs.depth_buffer[i]));
    }
    if(z_buffer)
        delete []z_buffer;
    z_buffer = new float[width*height];
    std::memcpy(z_buffer, rhs.z_buffer, width*height*sizeof(float));
}
void Framebuffers::clear(BufferType type)
{
    if(type==BufferType::COLOR)
        color_buffer->clear();
    else if(type==BufferType::DEPTH)
    {
        for(int i=0 ;i<depth_buffer.size(); i++)
            depth_buffer[i]->clear();
    }
    else if(type==(BufferType::COLOR|BufferType::DEPTH))
    {
        color_buffer->clear();
        for(int i=0 ;i<depth_buffer.size(); i++)
            depth_buffer[i]->clear();
    }
    std::fill(z_buffer, z_buffer+width*height, std::numeric_limits<float>::max());
}


}
