#pragma once

#include "Eigen/Eigen"
#include <memory>
#include <vector>
#include <array>
#include <random>
#include <iostream>
#include "buffer.hpp"
#include "mipmap.hpp"
#include "texture.hpp"

#define MY_PI 3.141592653589793
#define MY_2PI 6.283185307179586
#define INV_PI 0.31830988618
#define INV_2PI 0.15915494309

namespace openrenderer{

union Float32ToUint8
{
    float num;
    uint8_t arr[4];
};

void sampleFromHalfSphere(std::vector<Eigen::Vector3f>& samples, int sample_num=64);

float viewDepth2screenDepth(float view_depth, float zNear, float zFar);

Eigen::Vector3f reflect(Eigen::Vector3f wi, Eigen::Vector3f normal);

Eigen::Matrix4f scale(float rateX, float rateY, float rateZ);

Eigen::Matrix4f rotate(float angle, const Eigen::Vector3f& v);

Eigen::Matrix4f translate(const Eigen::Vector3f& v);

void decomposeTRS(const Eigen::Matrix4f& TRS, Eigen::Vector3f& scaleVec, Eigen::Vector3f& translateVec, Eigen::Matrix3f& rotateMat);

Eigen::Matrix4f lookAt(const Eigen::Vector3f& eyePos, const Eigen::Vector3f& center=Eigen::Vector3f(0.f, 0.f, 0.f), const Eigen::Vector3f& up=Eigen::Vector3f(0.f, 1.f, 0.f));

Eigen::Matrix4f ortho(float left, float right, float bottom, float top, float zNear, float zFar);

Eigen::Matrix4f perspective(float fovy, float aspect, float z_near, float z_far);

Eigen::Vector3f inverse2Dto3D(int x, int y, float z, int w, int h, const Eigen::Matrix4f& model, const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection);

void RT_decompose(const Eigen::Matrix4f& RT, Eigen::Matrix4f& R, Eigen::Matrix4f& T);

enum class LightType{ POINT, DIRECTION, SURFACE };
struct Light{
    LightType type;
    Eigen::Vector3f pos;
    Eigen::Vector3f intensity;
    Eigen::Vector3f direction;
    bool hasShadowMap;
    Eigen::Matrix4f lightVP;
    Buffer<uint8_t>* shadowMap;
};

struct Uniform{
    std::vector<Eigen::Matrix4f> models;
    Eigen::Matrix4f view;
    Eigen::Matrix4f projection;
    Eigen::Vector3f cameraPos;
    Eigen::Vector3f lookatPoint = Eigen::Vector3f::Zero();
    Eigen::Vector3f lightDir;
    std::vector<Light> lights;
    std::vector<Eigen::Vector3f> sampleFromHalfSphere;
    int width;
    int height;
    float zNear;
    float zFar;
    float shadowmap_zNear;
    float shadowmap_zFar;
    MipMap<float, 8>* depth_mipmap=nullptr;

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
        zNear = z_near;
        zFar = z_far;
        projection = perspective(fovy, aspect, z_near, z_far); 
    }
    void set_orthoProjection(float left, float right, float bottom, float top, float z_near, float z_far)   
    { 
        zNear = z_near;
        zFar = z_far;
        projection = ortho(left, right, bottom, top, z_near, z_far); 
    }
    void set_projection(Eigen::Matrix4f projectionMat) { projection = projectionMat; }
    void move_model(int index, float alpha, const Eigen::Vector3f& axis, const Eigen::Vector3f& trans)
    {
        alpha = alpha / 180.f * float(MY_PI);
        alpha = alpha > 360.0f ? 0.f : alpha;
        Eigen::Vector3f scaleVec;
        Eigen::Vector3f translateVec;
        Eigen::Matrix3f rotateMat;
        decomposeTRS(models[index], scaleVec, translateVec, rotateMat);
        Eigen::Matrix4f R;
        R << rotateMat(0,0), rotateMat(0,1), rotateMat(0,2), 0.f,
             rotateMat(1,0), rotateMat(1,1), rotateMat(1,2), 0.f,
             rotateMat(2,0), rotateMat(2,1), rotateMat(2,2), 0.f,
             0.f, 0.f, 0.f, 1.f;

        models[index] = translate(trans) * translate(translateVec) * rotate(alpha, axis) * R * scale(scaleVec.x(), scaleVec.y(), scaleVec.z());
    }
    void move_model(int index, const Eigen::Matrix4f& modelMat)
    {
        Eigen::Vector3f scaleVec0, scaleVec1;
        Eigen::Vector3f translateVec0, translateVec1;
        Eigen::Matrix3f rotateMat0, rotateMat1;
        Eigen::Matrix4f R0,R1;
        decomposeTRS(models[index], scaleVec0, translateVec0, rotateMat0);
        decomposeTRS(modelMat, scaleVec1, translateVec1, rotateMat1);
        R0 << rotateMat0(0,0), rotateMat0(0,1), rotateMat0(0,2), 0.f,
              rotateMat0(1,0), rotateMat0(1,1), rotateMat0(1,2), 0.f,
              rotateMat0(2,0), rotateMat0(2,1), rotateMat0(2,2), 0.f,
              0.f, 0.f, 0.f, 1.f;
        R1 << rotateMat1(0,0), rotateMat1(0,1), rotateMat1(0,2), 0.f,
              rotateMat1(1,0), rotateMat1(1,1), rotateMat1(1,2), 0.f,
              rotateMat1(2,0), rotateMat1(2,1), rotateMat1(2,2), 0.f,
              0.f, 0.f, 0.f, 1.f;
        models[index] = translate(translateVec1) * translate(translateVec0) * R1 * R0 * scale(scaleVec1.x(), scaleVec1.y(), scaleVec1.z()) * scale(scaleVec0.x(), scaleVec0.y(), scaleVec0.z());
    }
    void init(int width_, int height_, std::vector<Eigen::Matrix4f> modelMats, const Eigen::Vector3f& cameraPos_, float aspect, 
                const Eigen::Vector3f& lookat=Eigen::Vector3f::Zero(), const Eigen::Vector3f& up=Eigen::Vector3f(0.f, 1.f, 0.f), float fovy=45.f, float z_near=0.1f, float z_far=10.f, const Eigen::Vector3f& lightDir_={0.f, 0.f, 1.f}, const std::vector<Light>& lights_=std::vector<Light>())
    {
        width = width_;
        height = height_;
        zNear = z_near;
        zFar = z_far;
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
    Eigen::Vector3f ndcPos;
    Eigen::Vector3f modelPos;
};

struct Gbuffers;
struct Point{
    Eigen::Vector2i screen_pos;
    float z;    // the depth in world space
    Vertex v;
    int obj_id;
    Texture*  colorTexture = nullptr;
    Texture*  normalTexture = nullptr;
    Gbuffers* gbuffers = nullptr;
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
    std::unique_ptr<Buffer<uint8_t>> color_buffer;
    std::vector<Buffer<uint8_t>*>    depth_buffer;
    float*                  z_buffer = nullptr;


    Framebuffers(int w, int h, bool enable_color=true, bool enable_depth=false, PixelFormat color_format=PixelFormat::ARGB8888, PixelFormat depth_format=PixelFormat::ARGB8888);
    ~Framebuffers();
    Framebuffers(const Framebuffers& rhs);
    void clear(BufferType type);
    void clearZ();
};

struct  Gbuffers{
    int width;
    int height;
    std::unique_ptr<Buffer<float>> position_buffer;
    std::unique_ptr<Buffer<float>> normal_buffer;
    std::unique_ptr<Buffer<float>> albedo_buffer;

    Gbuffers() {  }
    Gbuffers(int w, int h, PixelFormat format=PixelFormat::ARGB8888);
    ~Gbuffers();
    Gbuffers(const Gbuffers& rhs);
    void clear();
};

}
