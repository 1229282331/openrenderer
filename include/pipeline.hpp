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
extern double t2;
extern double t3;
extern double t4;

namespace openrenderer{

struct Region{
    Eigen::Vector2i min;
    Eigen::Vector2i max;
    Region()
    {
        min = Eigen::Vector2i{ std::numeric_limits<int>::max(), std::numeric_limits<int>::max() };
        max = Eigen::Vector2i{ std::numeric_limits<int>::min(), std::numeric_limits<int>::min() };
    }
    Region(const Eigen::Vector2i& min_, const Eigen::Vector2i& max_)
    {
        min = min_;
        max = max_;
    }
    Region(const Region& rhs)
    {
        min = rhs.min;
        max = rhs.max;
    }
    void reset()
    {
        min = Eigen::Vector2i{ std::numeric_limits<int>::max(), std::numeric_limits<int>::max() };
        max = Eigen::Vector2i{ std::numeric_limits<int>::min(), std::numeric_limits<int>::min() };
    }
    Region Union(const Region& rhs)
    {
        Region ret;
        ret.min = { std::min(min.x(), rhs.min.x()), std::min(min.y(), rhs.min.y()) };
        ret.max = { std::max(max.x(), rhs.max.x()), std::max(max.y(), rhs.max.y()) };
        return ret;
    }
    Region& operator=(const Region& rhs)
    {
        min = rhs.min;
        max = rhs.max;
        return *this;
    }
};

enum class PrimitiveType{ POINT=1, LINE, TRIANGLE };
enum class ShadeFrequency{ FLAT, GOURAUD };

class Pipeline{
public:
    Pipeline(PrimitiveType primitive, ShadeFrequency freq, std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)> vertexShaderFunc, std::function<Eigen::Vector3f(const Point&)> fragmentShaderFunc, const Framebuffers& framebuffers, const Gbuffers* gbuffers) 
        : m_primitiveType(primitive), m_shadeFrequency(freq), m_vertexShaderFunc(vertexShaderFunc), m_fragmentShaderFunc(fragmentShaderFunc), m_colorTexture(nullptr), m_renderRegion(Region())
    { 

        m_framebuffers = std::make_unique<Framebuffers>(framebuffers);
        if(gbuffers)
            m_gbuffers = std::make_unique<Gbuffers>(*gbuffers);
        clear(); 
    }
    ~Pipeline();
    void clear();
    void update(Vertex v);
    void update(Vertex v0, Vertex v1, int idx0, int idx1);
    void update(Vertex v0, Vertex v1, Vertex v2, int idx0, int idx1, int idx2);
    void set_state(std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)> vertexShader, std::function<Eigen::Vector3f(const Point&)> fragmentShader, Texture* colorTexture, Texture* normalTexture=nullptr, Gbuffers* defferedGbuffers=nullptr, int max_rasterSize=3000, PrimitiveType primitive=PrimitiveType::TRIANGLE, ShadeFrequency freq=ShadeFrequency::FLAT);
    void run(int obj_id);
    void generate_gbuffers(int obj_id);
    void generate_shadowmap(int obj_id, int shadowmap_id);

    Region&        renderRegion() { return m_renderRegion; }
    PrimitiveType primitiveType() { return m_primitiveType; }
    Framebuffers* framebuffers() { return m_framebuffers.get(); }
    Gbuffers*     gbuffers() { return m_gbuffers.get(); }

    void set_vertexShader(std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)> vertexShader) { m_vertexShaderFunc = vertexShader; }
    void set_fragmentShader(std::function<Eigen::Vector3f(const Point&)> fragmentShader) { m_fragmentShaderFunc = fragmentShader; }

private:
    Eigen::Vector4f                                                        gl_Position[3];

    Region                                                                 m_renderRegion;
    PrimitiveType                                                          m_primitiveType;
    ShadeFrequency                                                         m_shadeFrequency;
    Texture*                                                               m_colorTexture;
    Texture*                                                               m_normalTexture;
    Gbuffers*                                                              m_defferedGbuffers;
    std::unique_ptr<Framebuffers>                                          m_framebuffers;
    std::unique_ptr<Gbuffers>                                              m_gbuffers;
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

};

Eigen::Vector3f barycentric(const Point* vs, const Eigen::Vector2i& p);

bool test_depth(int x, int y, float z, int width, int height, float* z_buffer);

float correction(Eigen::Vector3f& bc, const Eigen::Vector4f* gl_Position);

int line(const Point* input, Point* raster_region, int max_size, int width, int height, Region& renderRegion);

int triangle(const Point* input, const Eigen::Vector4f* gl_Position, Point* raster_region, int max_size, ShadeFrequency freq, int width, int height, Region& renderRegion);

int thread_schedule(int thread_id, int i, int total_threads);

Eigen::Vector3f calculate_tangent(const Vertex points[3]);


}