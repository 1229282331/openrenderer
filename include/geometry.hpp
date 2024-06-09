#pragma once

#include "Eigen/Eigen"
#include <memory>
#include <vector>
#include <array>
#include <iostream>
#include "buffer.hpp"
#include "texture.hpp"

#define MY_PI 3.1415926


namespace openrenderer{

union Float32ToUint8
{
    float num;
    uint8_t arr[4];
};


Eigen::Matrix4f scale(float rateX, float rateY, float rateZ);

Eigen::Matrix4f rotate(float angle, const Eigen::Vector3f& v);

Eigen::Matrix4f translate(const Eigen::Vector3f& v);

Eigen::Matrix4f lookAt(const Eigen::Vector3f& eyePos, const Eigen::Vector3f& center=Eigen::Vector3f(0.f, 0.f, 0.f), const Eigen::Vector3f& up=Eigen::Vector3f(0.f, 1.f, 0.f));

Eigen::Matrix4f ortho(float left, float right, float bottom, float top, float zNear, float zFar);

Eigen::Matrix4f perspective(float fovy, float aspect, float z_near, float z_far);

Eigen::Vector3f inverse2Dto3D(int x, int y, float z, int w, int h, const Eigen::Matrix4f& model, const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection);

void RT_decompose(const Eigen::Matrix4f& RT, Eigen::Matrix4f& R, Eigen::Matrix4f& T);

struct Light{
    Eigen::Vector3f pos;
    Eigen::Vector3f intensity;
    bool hasShadowMap;
    Eigen::Matrix4f lightVP;
    Buffer* shadowMap;
};

struct Uniform{
    std::vector<Eigen::Matrix4f> models;
    Eigen::Matrix4f view;
    Eigen::Matrix4f projection;
    Eigen::Vector3f cameraPos;
    Eigen::Vector3f lightDir;
    std::vector<Light> lights;
    int width;
    int height;

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
    void set_projection(float fovy, float aspect, float z_near, float z_far)   
    { 
        projection = perspective(fovy, aspect, z_near, z_far); 
    }
    void set_orthoProjection(float left, float right, float bottom, float top, float z_near, float z_far)   
    { 
        projection = ortho(left, right, bottom, top, z_near, z_far); 
    }
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
    void init(int width_, int height_, std::vector<Eigen::Matrix4f> modelMats, const Eigen::Vector3f& cameraPos_, float aspect, 
                const Eigen::Vector3f& lookat=Eigen::Vector3f::Zero(), const Eigen::Vector3f& up=Eigen::Vector3f(0.f, 1.f, 0.f), float fovy=45.f, float z_near=0.1f, float z_far=10.f, const Eigen::Vector3f& lightDir_={0.f, 0.f, 1.f}, const std::vector<Light>& lights_=std::vector<Light>())
    {
        width = width_;
        height = height_;
        cameraPos = cameraPos_;
        set_models(modelMats);
        set_view(cameraPos_, lookat, up);
        set_projection(fovy, aspect, z_near, z_far);
        lightDir = lightDir_.normalized();
        lights = lights_;
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
    Eigen::Vector3f tangent;
    Texture* normalTexture = nullptr;
};

struct vertex_shader_out{
    Eigen::Vector3f normal;
    Eigen::Matrix3f TBN;
    Eigen::Vector3f position;
};

struct Point{
    Eigen::Vector2i screen_pos;
    float z;
    Vertex v;
    int obj_id;
    Texture *colorTexture = nullptr;
    Texture *normalTexture = nullptr;
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

struct Framebuffers{
    int width;
    int height;
    std::unique_ptr<Buffer> color_buffer;
    std::vector<Buffer*>    depth_buffer;
    float*                  z_buffer = nullptr;


    Framebuffers(int w, int h, bool enable_color=true, bool enable_depth=false, PixelFormat color_format=PixelFormat::ARGB8888, PixelFormat depth_format=PixelFormat::GRAY8);
    ~Framebuffers();
    Framebuffers(const Framebuffers& rhs);
    void clear(BufferType type);
    void clearZ();
};

}
