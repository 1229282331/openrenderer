#include "renderer.hpp"



namespace openrenderer{

Render::Render(int w, int h, bool enable_color, bool enable_depth, PixelFormat color_format, PixelFormat depth_format) : m_width(w), m_height(h)
{
    m_framebuffers = std::make_unique<Framebuffers>(w, h, enable_color, enable_depth, color_format, depth_format);
}

Render::~Render()
{
    m_framebuffers.reset();
    m_pipeline.reset();
}

void Render::init_pipeline(PrimitiveType primitive, std::function<Eigen::Vector3f(const vertex_shader_in&, vertex_shader_out&)> vertexShaderFunc, std::function<Eigen::Vector3f(const Point&)> fragmentShaderFunc)
{
    m_pipeline = std::make_unique<Pipeline>(primitive, vertexShaderFunc, fragmentShaderFunc, m_framebuffers.get());
}


void Render::drawFrame(const Loader& obj_loader)
{
    static auto startTime = clock();
    auto curTime = clock();
    float dt = (curTime-startTime)/1000.f;
    if(!m_pipeline)
    {
        printf("[error] pipeline hasn't set!\n");
        return;
    }

    m_framebuffers->clear(BufferType::COLOR);
    m_framebuffers->clear(BufferType::DEPTH);

    for(auto& obj : obj_loader.objects)
    {
        m_pipeline->set_state(obj.vertexShader, obj.fragmentShader, m_width*m_height/int(obj.indices.size())*3*500, PrimitiveType::TRIANGLE);
        if(m_pipeline->primitiveType()==PrimitiveType::POINT)
            for(int i=0; i<obj.vertices.size(); i++)
            {
                m_pipeline->update(obj.vertices[i]);
                m_pipeline->run(obj.id);
            }
        else if(m_pipeline->primitiveType()==PrimitiveType::LINE)
            for(int i=0; i<obj.indices.size(); i+=3)
            {
                m_pipeline->update(obj.vertices[obj.indices[i]], obj.vertices[obj.indices[i+1]], obj.indices[i], obj.indices[i+1]); //line v0v1
                m_pipeline->run(obj.id);
                m_pipeline->update(obj.vertices[obj.indices[i+1]], obj.vertices[obj.indices[i+2]], obj.indices[i+1], obj.indices[i+2]); //line v1v2
                m_pipeline->run(obj.id);
                m_pipeline->update(obj.vertices[obj.indices[i+2]], obj.vertices[obj.indices[i]], obj.indices[i+2], obj.indices[i]); //line v2v0
                m_pipeline->run(obj.id);
            }
        else if(m_pipeline->primitiveType()==PrimitiveType::TRIANGLE)
            for(int i=0; i<obj.indices.size(); i+=3)
            {
                ubo.faceNormal = (obj.vertices[obj.indices[i+2]].pos-obj.vertices[obj.indices[i]].pos).cross(obj.vertices[obj.indices[i+1]].pos-obj.vertices[obj.indices[i]].pos).normalized();
                m_pipeline->update(obj.vertices[obj.indices[i]], obj.vertices[obj.indices[i+1]], obj.vertices[obj.indices[i+2]], obj.indices[i], obj.indices[i+1], obj.indices[i+2]);
                m_pipeline->run(obj.id);
            }
    }
    for(int i=0; i<m_framebuffers->width*m_framebuffers->height; i++)
    {
        int gray_value = int(m_framebuffers->z_buffer[i] * 255.f);
        m_framebuffers->depth_buffer->buffer[i*3] = gray_value;
        m_framebuffers->depth_buffer->buffer[i*3+1] = gray_value;
        m_framebuffers->depth_buffer->buffer[i*3+2] = gray_value;
    }
}


}