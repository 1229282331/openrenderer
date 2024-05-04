#pragma once

#include "Eigen/Eigen"
#include <vector>
#include <array>
#include <iostream>

#define MY_PI 3.1415926

namespace openrenderer{
enum class PixelFormat{ RGB888=1, ARGB8888, GRAY8 };
enum class ColorBit{ B=0, G, R, A };
enum class BufferType{ COLOR=1, DEPTH };

inline Eigen::Matrix4f rotate(float angle, const Eigen::Vector3f& v)
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

inline Eigen::Matrix4f translate(const Eigen::Vector3f& v)
{
    Eigen::Matrix4f Translate = Eigen::Matrix4f::Identity();
    Translate(0, 3) = v[0];
    Translate(1, 3) = v[1];
    Translate(2, 3) = v[2];
    Translate(3, 3) = 1.f;

    return Translate;
}

inline Eigen::Matrix4f lookAt(const Eigen::Vector3f& eyePos, const Eigen::Vector3f& center=Eigen::Vector3f(0.f, 0.f, 0.f), const Eigen::Vector3f& up=Eigen::Vector3f(0.f, 1.f, 0.f))
{
    Eigen::Vector3f const f((center - eyePos).normalized());
    Eigen::Vector3f const s((f.cross(up)).normalized());
    Eigen::Vector3f const u((s.cross(f)).normalized());

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

inline Eigen::Matrix4f perspective(float fovy, float aspect, float z_near, float z_far)
{
    // assert(abs(aspect - std::numeric_limits<float>::epsilon()) > 0.f);

    float const tanHalfFovy = tan(fovy / 2.f);

    Eigen::Matrix4f Result = Eigen::Matrix4f::Identity();
    Result(0, 0) = 1.f / (aspect * tanHalfFovy);
    Result(1, 1) = 1.f / (tanHalfFovy);
    Result(2, 2) = z_far / (z_near - z_far);
    Result(2, 3) = - 1.f;
    Result(3, 2) = -(z_far * z_near) / (z_far - z_near);
    return Result.transpose();
}

inline Eigen::Vector3f inverse2Dto3D(int x, int y, float z, int w, int h, const Eigen::Matrix4f& model, const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection)
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

inline void RT_decompose(const Eigen::Matrix4f& RT, Eigen::Matrix4f& R, Eigen::Matrix4f& T)
{
    R = RT;
    R.col(3) = Eigen::Vector4f(0.f, 0.f, 0.f, 1.f);
    T = Eigen::Matrix4f::Identity();
    T.col(3) = RT.col(3);
}


struct Uniform{
    std::vector<Eigen::Matrix4f> models;
    Eigen::Matrix4f view;
    Eigen::Matrix4f projection;
    Eigen::Vector3f cameraPos;
    Eigen::Vector3f lightDir;
    Eigen::Vector3f faceNormal;

    void set_models(std::vector<Eigen::Matrix4f> modelMats) { models = modelMats; }
    void set_model(int index, float alpha, const Eigen::Vector3f& axis, const Eigen::Vector3f& trans) 
    { 
        alpha = alpha / 180.f * float(MY_PI);
        alpha = alpha > 360.0f ? 0.f : alpha;
        models[index] = translate(trans) * rotate(alpha, axis);
    }
    void set_model(int index, Eigen::Matrix4f modelMat) { models[index] = modelMat; }
    void set_view(const Eigen::Vector3f& cameraPos, const Eigen::Vector3f& lookat, const Eigen::Vector3f& up) { view = lookAt(cameraPos, lookat, up); }
    void set_view(Eigen::Matrix4f viewMat) { view = viewMat; }
    void set_projection(float fovy, float aspect, float z_near, float z_far)   { projection = perspective(fovy, aspect, z_near, z_far); }
    void set_projection(Eigen::Matrix4f projectionMat) { projection = projectionMat; }
    void move_model(int index, float alpha, const Eigen::Vector3f& axis, const Eigen::Vector3f& trans)
    {
        alpha = alpha / 180.f * float(MY_PI);
        alpha = alpha > 360.0f ? 0.f : alpha;
        Eigen::Matrix4f R, T;
        RT_decompose(models[index], R, T);
        models[index] = T * translate(trans) * rotate(alpha, axis) * R;
    }
    void move_model(int index, const Eigen::Matrix4f& modelMat)
    {
        Eigen::Matrix4f R0, T0;
        Eigen::Matrix4f R1, T1;
        RT_decompose(models[index], R0, T0);
        RT_decompose(modelMat, R1, T1);
        
        models[index] = T1 * T0 * R1 * R0;
    }
    void init(std::vector<Eigen::Matrix4f> modelMats, const Eigen::Vector3f& cameraPos_, float aspect, 
                const Eigen::Vector3f& lookat=Eigen::Vector3f::Zero(), const Eigen::Vector3f& up=Eigen::Vector3f(0.f, 1.f, 0.f), float fovy=45.f, float z_near=0.1f, float z_far=10.f, const Eigen::Vector3f& lightDir_={0.f, 0.f, 1.f})
    {
        cameraPos = cameraPos_;
        set_models(modelMats);
        set_view(cameraPos_, lookat, up);
        set_projection(fovy, aspect, z_near, z_far);
        lightDir = lightDir_;
    }
};

struct Vertex{
    Eigen::Vector3f pos;
    Eigen::Vector2f texCoord;
    Eigen::Vector3f normal;
    Eigen::Vector3f color;

    Vertex() : pos(Eigen::Vector3f::Zero()), texCoord(Eigen::Vector2f::Zero()), normal(Eigen::Vector3f::Zero()), color(Eigen::Vector3f::Zero()) {  }

    bool operator==(const Vertex& rhs) const 
    {
        return pos==rhs.pos && texCoord==rhs.texCoord && normal==rhs.normal && color==rhs.color;
    }
};

struct vertex_shader_in{
    Vertex vertex;
    int obj_id;
};

struct vertex_shader_out{
    Eigen::Vector2i screenPos;
};

struct Point{
    Eigen::Vector2i screen_pos;
    float z;
    Vertex v;
    vertex_shader_out attrs;
};

struct Line{
    std::array<Point, 2> v;
    std::array<int, 2> index;
};

struct Triangle{
    std::array<Point, 3> v;
    std::array<int, 3> index;
};


struct Buffer{
    uint8_t* buffer;
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
        buffer = new uint8_t[size];
        memcpy(buffer, rhs.buffer, sizeof(uint8_t)*size);
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
        buffer = new uint8_t[size];
        memset(buffer, 0, size);
    }
    ~Buffer() { delete []buffer; }
    uint8_t& operator()(int i, int j, ColorBit bit) { return buffer[pbyte*(i*width+j)+int(bit)];  } 
    uint8_t  get(int i, int j, ColorBit bit) { return buffer[pbyte*(i*width+j)+int(bit)];  } 
    void     set(int i, int j, ColorBit bit, uint8_t value) { buffer[pbyte*(i*width+j)+int(bit)]=value; }
    void     clear() { memset(buffer, 0, size); }

};

struct Framebuffers{
    int width;
    int height;
    std::unique_ptr<Buffer> color_buffer;
    std::unique_ptr<Buffer> depth_buffer;
    float*                  z_buffer = nullptr;


    Framebuffers(int w, int h, bool enable_color=true, bool enable_depth=false, PixelFormat color_format=PixelFormat::ARGB8888, PixelFormat depth_format=PixelFormat::GRAY8)
        : width(w), height(h)
    {
        if(enable_color)
            color_buffer = std::make_unique<Buffer>(color_format, w, h);
        if(enable_depth)
        {
            depth_buffer = std::make_unique<Buffer>(depth_format, w, h);
            memset(depth_buffer->buffer, 255, depth_buffer->size);
        }
        z_buffer = new float[width*height];
        std::fill(z_buffer, z_buffer+width*height, 1.f);
    }
    ~Framebuffers()
    {
        delete []z_buffer;
        color_buffer.reset();
        depth_buffer.reset();
    }
    void clear(BufferType type)
    {
        switch (type)
        {
        case BufferType::COLOR:
            color_buffer->clear();
            break;
        case BufferType::DEPTH:
            depth_buffer->clear();
            break;
        default:
            break;
        }
        std::fill(z_buffer, z_buffer+width*height, 1.f);
    }
    
};


}