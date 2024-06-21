#pragma once

#include "geometry.hpp"

extern openrenderer::Uniform ubo;

namespace openrenderer{

void sampleFromHalfSphere(std::vector<Eigen::Vector3f>& samples, int sample_num)
{
    std::uniform_real_distribution<float> randomFloats(0.f, 1.f);
    std::default_random_engine generator;
    std::function<float(float, float, float)> lerp = [](float a, float b, float f){ return a+f*(b-a); };
    for (int i = 0; i < sample_num; ++i)
    {
        Eigen::Vector3f point(
            randomFloats(generator) * 2.0f - 1.0f, 
            randomFloats(generator) * 2.0f - 1.0f, 
            randomFloats(generator)
        );
        point = point.normalized();
        point *= randomFloats(generator);
        float scale = i / 64.0f; 
        scale = lerp(0.1f, 1.0f, scale * scale);
        point *= scale;
        samples.push_back(point);  
    }
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

Eigen::Matrix4f ortho(float left, float right, float bottom, float top, float zNear, float zFar)
{
    // assert(abs(aspect - std::numeric_limits<float>::epsilon()) > 0.f);

    Eigen::Matrix4f res = Eigen::Matrix4f::Identity();
    res(0, 0) = 2.f / (right - left);
    res(1, 1) = 2.f / (top - bottom);
    res(2, 2) = - 1.f / (zFar - zNear);
    res(0, 3) = - (right + left) / (right - left);
    res(1, 3) = - (top + bottom) / (top - bottom);
    res(2, 3) = - zNear / (zFar - zNear);
    return res;
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


Framebuffers::Framebuffers(int w, int h, bool enable_color, bool enable_depth, PixelFormat color_format, PixelFormat depth_format) : width(w), height(h)
{
    if(enable_color)
        color_buffer = std::make_unique<Buffer<uint8_t>>(color_format, w, h);
    if(enable_depth)
    {
        int shadowMap_num = 0;
        for(int i=0; i<ubo.lights.size(); i++)
        {
            if(ubo.lights[i].hasShadowMap)
            {
                Buffer<uint8_t>* buf = new Buffer<uint8_t>(depth_format, w, h);
                depth_buffer.push_back(buf);
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
    {
        delete depth_buffer[i];
        depth_buffer[i] = nullptr;
    }
    depth_buffer.clear();
}
Framebuffers::Framebuffers(const Framebuffers& rhs)
{
    width = rhs.width;
    height = rhs.height;
    if(color_buffer.get())
        color_buffer.reset();
    color_buffer = std::make_unique<Buffer<uint8_t>>(*rhs.color_buffer);
    if(depth_buffer.size())
    {
        for(int i=0; i<depth_buffer.size(); i++)
        {
            delete depth_buffer[i];
            depth_buffer[i] = nullptr;
        }
        depth_buffer.clear();
    }
    for(int i=0; i<rhs.depth_buffer.size(); i++)
    {
        Buffer<uint8_t>* buf = new Buffer<uint8_t>(*rhs.depth_buffer[i]);
        depth_buffer.push_back(buf);
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
void Framebuffers::clearZ()
{
    std::fill(z_buffer, z_buffer+width*height, std::numeric_limits<float>::max());
}

Gbuffers::Gbuffers(int w, int h, PixelFormat format)
{
    position_buffer = std::make_unique<Buffer<float>>(format, w, h);
    normal_buffer = std::make_unique<Buffer<float>>(format, w, h);
    albedo_buffer = std::make_unique<Buffer<float>>(format, w, h);
}
Gbuffers::~Gbuffers()
{
    position_buffer.reset();
    normal_buffer.reset();
    albedo_buffer.reset();
}
Gbuffers::Gbuffers(const Gbuffers& rhs)
{
    width = rhs.width;
    height = rhs.height;
    position_buffer.reset();
    normal_buffer.reset();
    albedo_buffer.reset();
    position_buffer = std::make_unique<Buffer<float>>(*rhs.position_buffer);
    normal_buffer = std::make_unique<Buffer<float>>(*rhs.normal_buffer);
    albedo_buffer = std::make_unique<Buffer<float>>(*rhs.albedo_buffer);
}
void Gbuffers::clear()
{
    position_buffer->clear();
    normal_buffer->clear();
    albedo_buffer->clear();
}


}
