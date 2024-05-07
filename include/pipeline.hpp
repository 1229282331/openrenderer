#pragma once

#include <iostream>
#include <functional>
#include "geometry.hpp"
#include "shader.hpp"

extern openrenderer::Uniform ubo;

namespace openrenderer{

enum class PrimitiveType{ POINT=1, LINE, TRIANGLE };
enum class ShadeFrequency{ FLAT, GOURAUD };

class Pipeline{
public:
    Pipeline(PrimitiveType primitive, ShadeFrequency freq, std::function<Eigen::Vector3f(const vertex_shader_in&, vertex_shader_out&)> vertexShaderFunc, std::function<Eigen::Vector3f(const Point&)> fragmentShaderFunc, Framebuffers* framebuffers) 
        : m_primitiveType(primitive), m_shadeFrequency(freq), m_vertexShaderFunc(vertexShaderFunc), m_fragmentShaderFunc(fragmentShaderFunc), m_framebuffers(framebuffers)
    { 
        clear(); 
    }
    ~Pipeline();
    void clear();
    void update(Vertex v);
    void update(Vertex v0, Vertex v1, int idx0, int idx1);
    void update(Vertex v0, Vertex v1, Vertex v2, int idx0, int idx1, int idx2);
    void set_state(std::function<Eigen::Vector3f(const vertex_shader_in&, vertex_shader_out&)> vertexShader, std::function<Eigen::Vector3f(const Point&)> fragmentShader, int max_rasterSize=3000, PrimitiveType primitive=PrimitiveType::TRIANGLE, ShadeFrequency freq=ShadeFrequency::FLAT);
    void run(int obj_id);

    PrimitiveType primitiveType() { return m_primitiveType; }



private:
    PrimitiveType                                                          m_primitiveType;
    ShadeFrequency                                                         m_shadeFrequency;
    Framebuffers*                                                          m_framebuffers;
    Vertex                                                                 m_vertexs[3];
    vertex_shader_out                                                      m_attrs[3];
    int                                                                    m_indices[3];
    Point                                                                  m_primitve[3];
    Point*                                                                 m_rasterPoints = nullptr;
    int                                                                    m_rasterSize;
    int                                                                    m_maxRasterSize;
    std::function<Eigen::Vector3f(const vertex_shader_in&, vertex_shader_out&)> m_vertexShaderFunc;
    std::function<Eigen::Vector3f(const Point&)>                           m_fragmentShaderFunc;
    


    Eigen::Vector3f vertex_shader(const vertex_shader_in& intput, vertex_shader_out& output);
    void primitive_assembly();
    void rasterization();
    Eigen::Vector3f fragment_shader(const Point& point_input);

    bool test_depth(int x, int y, float z, int width, int height, float* z_buffer);
};


int line(const Point* input, Point* raster_region, int max_size);

int triangle(const Point* input, Point* raster_region, int max_size, ShadeFrequency freq);

}