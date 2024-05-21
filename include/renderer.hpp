#pragma once

#include <ctime>
#include <omp.h>
#include "geometry.hpp"
#include "loader.hpp"
#include "pipeline.hpp"
#include "shader.hpp"

namespace openrenderer{

class Render{
public:
    Render(int w, int h, bool enable_color=true, bool enable_depth=false, PixelFormat color_format=PixelFormat::ARGB8888, PixelFormat depth_format=PixelFormat::GRAY8);
    ~Render();

    void init_pipeline(PrimitiveType primitive, ShadeFrequency freq, std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)> vertexShaderFunc, std::function<Eigen::Vector3f(const Point&)> fragmentShaderFunc);
    void drawFrame(const Loader& obj_loader);

    int width() const { return m_width; }
    int height() const { return m_height; }
    Pipeline& pipeline(int index) { return *m_pipeline[index]; }
    Framebuffers* framebuffers() { return m_framebuffers.get(); }


private:
    int m_width;
    int m_height;
    static const int num_threads = 20;

    std::unique_ptr<Framebuffers> m_framebuffers;
    std::unique_ptr<Pipeline> m_pipeline[num_threads];

};


}