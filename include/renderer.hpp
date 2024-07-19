#pragma once

#include <ctime>
#include <omp.h>
#include "geometry.hpp"
#include "loader.hpp"
#include "pipeline.hpp"
#include "shader.hpp"

extern double t0;
extern double t1;
extern double t2;
extern double t3;

namespace openrenderer{

class Render{
public:
    Render(int w, int h, bool enable_color=true, bool enable_depth=false, bool enable_defferedRender=false, PixelFormat color_format=PixelFormat::ARGB8888, PixelFormat depth_format=PixelFormat::ARGB8888, PixelFormat gbuffer_format=PixelFormat::ARGB8888);
    ~Render();

    void init_pipeline(PrimitiveType primitive, ShadeFrequency freq, std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)> vertexShaderFunc, std::function<Eigen::Vector3f(const Point&)> fragmentShaderFunc);
    void drawFrame(const Loader& obj_loader);

    int width() const { return m_width; }
    int height() const { return m_height; }
    Pipeline& pipeline(int index) { return *m_pipeline[index]; }
    Framebuffers* framebuffers() { return m_framebuffers.get(); }

    static const int num_threads = 16;
private:
    int m_width;
    int m_height;
    int m_isDefferedRender;

    std::unique_ptr<Framebuffers> m_framebuffers;
    std::unique_ptr<Gbuffers> m_gbuffers;
    std::unique_ptr<Pipeline> m_pipeline[num_threads];

    void linkSubFramebuffers(int use_pipelineNum, BufferType type=BufferType::COLOR, int buf_id=0);
    void linkSubGbuffers(int use_pipelineNum);

};


}