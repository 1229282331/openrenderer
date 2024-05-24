#pragma once

#include <iostream>
#include <functional>
#include <cmath>
#include <chrono>
#include "geometry.hpp"
#include "shader.hpp"
#include "texture.hpp"

extern openrenderer::Uniform ubo;

extern double t0;
extern double t1;

namespace openrenderer{

enum class PrimitiveType{ POINT=1, LINE, TRIANGLE };
enum class ShadeFrequency{ FLAT, GOURAUD };

class Pipeline{
public:
    Pipeline(PrimitiveType primitive, ShadeFrequency freq, std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)> vertexShaderFunc, std::function<Eigen::Vector3f(const Point&)> fragmentShaderFunc, Framebuffers* framebuffers) 
        : m_primitiveType(primitive), m_shadeFrequency(freq), m_vertexShaderFunc(vertexShaderFunc), m_fragmentShaderFunc(fragmentShaderFunc), m_framebuffers(framebuffers), m_colorTexture(nullptr)
    { 
        clear(); 
    }
    ~Pipeline();
    void clear();
    void update(Vertex v);
    void update(Vertex v0, Vertex v1, int idx0, int idx1);
    void update(Vertex v0, Vertex v1, Vertex v2, int idx0, int idx1, int idx2);
    void set_state(std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)> vertexShader, std::function<Eigen::Vector3f(const Point&)> fragmentShader, Texture* colorTexture, Texture* normalTexture=nullptr, int max_rasterSize=3000, PrimitiveType primitive=PrimitiveType::TRIANGLE, ShadeFrequency freq=ShadeFrequency::FLAT);
    void run(int obj_id);

    PrimitiveType primitiveType() { return m_primitiveType; }



private:
    Eigen::Vector4f                                                        gl_Position[3];

    PrimitiveType                                                          m_primitiveType;
    ShadeFrequency                                                         m_shadeFrequency;
    Texture*                                                               m_colorTexture;
    Texture*                                                               m_normalTexture;
    Framebuffers*                                                          m_framebuffers;
    Vertex                                                                 m_vertexs[3];
    vertex_shader_out                                                      m_attrs[3];
    int                                                                    m_indices[3];
    Point                                                                  m_primitive[3];
    Point*                                                                 m_rasterPoints = nullptr;
    int                                                                    m_rasterSize;
    int                                                                    m_maxRasterSize;
    std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)> m_vertexShaderFunc;
    std::function<Eigen::Vector3f(const Point&)>                           m_fragmentShaderFunc;
    


    Eigen::Vector4f vertex_shader(const vertex_shader_in& intput, vertex_shader_out& output);
    void primitive_assembly();
    void rasterization();
    Eigen::Vector3f fragment_shader(const Point& point_input);

    bool test_depth(int x, int y, float z, int width, int height, float* z_buffer);

    void subRender(int begin, int end);
};

Eigen::Vector3f barycentric(const Point* vs, const Eigen::Vector2i& p);

float correction(Eigen::Vector3f& bc, const Eigen::Vector4f* gl_Position);

int line(const Point* input, Point* raster_region, int max_size, int width, int height);

int triangle(const Point* input, const Eigen::Vector4f* gl_Position, Point* raster_region, int max_size, ShadeFrequency freq, int width, int height);

int thread_schedule(int thread_id, int i, int total_threads);

Eigen::Vector3f calculate_tangent(const Vertex points[3]);

}