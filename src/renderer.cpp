#include "renderer.hpp"



namespace openrenderer{

Render::Render(int w, int h, bool enable_color, bool enable_depth, PixelFormat color_format, PixelFormat depth_format) : m_width(w), m_height(h)
{
    m_framebuffers = std::make_unique<Framebuffers>(w, h, enable_color, enable_depth, color_format, depth_format);
}

Render::~Render()
{
    m_framebuffers.reset();
    for(int i=0; i<num_threads; i++)
        m_pipeline[i].reset();
}

void Render::init_pipeline(PrimitiveType primitive, ShadeFrequency freq, 
                            std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)> vertexShaderFunc, 
                            std::function<Eigen::Vector3f(const Point&)> fragmentShaderFunc)
{
    for(int i=0; i<num_threads; i++)
        m_pipeline[i] = std::make_unique<Pipeline>(primitive, freq, vertexShaderFunc, fragmentShaderFunc, *m_framebuffers);
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

    auto start = std::chrono::high_resolution_clock::now();
    #pragma omp parallel for num_threads(num_threads)
    for(int i=0; i<num_threads; i++)
        m_pipeline[i]->framebuffers()->clear(BufferType::COLOR);
    m_framebuffers->clear(BufferType::COLOR);
    // m_framebuffers->clear(BufferType::DEPTH);
    auto end = std::chrono::high_resolution_clock::now();
    t0 += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-6;

    for(auto& obj : obj_loader.objects)
    {
        int use_threads = std::clamp(int(obj.indices.size())/3, 1, num_threads);

        start = std::chrono::high_resolution_clock::now();
        for(int i=0; i<num_threads; i++)
        {
            m_pipeline[i]->set_state(obj.vertexShader, obj.fragmentShader, obj.colorTexture, obj.normalTexture, std::min(m_width*m_height/int(obj.indices.size())*3*1000, m_width*m_height), PrimitiveType::TRIANGLE, ShadeFrequency::GOURAUD);
        }
        end = std::chrono::high_resolution_clock::now();
        t1 += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-6;

        if(m_pipeline[0]->primitiveType()==PrimitiveType::POINT)
        {
            #pragma omp parallel for num_threads(use_threads)
            for(int i=0; i<obj.vertices.size(); i++)
            {
                int thread_id = omp_get_thread_num();
                m_pipeline[thread_id]->update(obj.vertices[i]);
                m_pipeline[thread_id]->run(obj.id);
            }

            linkSubFramebuffers();
        }
        else if(m_pipeline[0]->primitiveType()==PrimitiveType::LINE)
        {
            #pragma omp parallel for num_threads(use_threads)
            for(int i=0; i<obj.indices.size(); i+=3)
            {
                int thread_id = omp_get_thread_num();
                m_pipeline[thread_id]->update(obj.vertices[obj.indices[i]], obj.vertices[obj.indices[i+1]], obj.indices[i], obj.indices[i+1]); //line v0v1
                m_pipeline[thread_id]->run(obj.id);
                m_pipeline[thread_id]->update(obj.vertices[obj.indices[i+1]], obj.vertices[obj.indices[i+2]], obj.indices[i+1], obj.indices[i+2]); //line v1v2
                m_pipeline[thread_id]->run(obj.id);
                m_pipeline[thread_id]->update(obj.vertices[obj.indices[i+2]], obj.vertices[obj.indices[i]], obj.indices[i+2], obj.indices[i]); //line v2v0
                m_pipeline[thread_id]->run(obj.id);
            }
            linkSubFramebuffers();
        }
        else if(m_pipeline[0]->primitiveType()==PrimitiveType::TRIANGLE)
        {
            auto start = std::chrono::high_resolution_clock::now();
            #pragma omp parallel for num_threads(use_threads)
            for(int i=0; i<obj.indices.size(); i+=3)
            {
                int thread_id = omp_get_thread_num();
                m_pipeline[thread_id]->update(obj.vertices[obj.indices[i]], obj.vertices[obj.indices[i+1]], obj.vertices[obj.indices[i+2]], obj.indices[i], obj.indices[i+1], obj.indices[i+2]);
                m_pipeline[thread_id]->run(obj.id);
            }
            auto end = std::chrono::high_resolution_clock::now();
            t2 += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-6;

            start = std::chrono::high_resolution_clock::now();
            linkSubFramebuffers();
            end = std::chrono::high_resolution_clock::now();
            t3 += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-6;
        }
    }
    
    // for(int i=0; i<m_framebuffers->width*m_framebuffers->height; i++)
    // {
    //     int gray_value = int(m_framebuffers->z_buffer[i] * 255.f);
    //     m_framebuffers->depth_buffer->buffer[i*3] = gray_value;
    //     m_framebuffers->depth_buffer->buffer[i*3+1] = gray_value;
    //     m_framebuffers->depth_buffer->buffer[i*3+2] = gray_value;
    // }

}

void Render::linkSubFramebuffers()
{
    Region region;
    for(int i=0; i<num_threads; i++)
        region = region.Union(m_pipeline[i]->renderRegion());

    #pragma omp parallel for num_threads(16)
    for(int x=region.min.x(); x<=region.max.x(); x++)
        for(int y=region.min.y(); y<=region.max.y(); y++)
        {
            float min_z = std::numeric_limits<float>::max();
            int thread_id = 0;
            int index = y*m_width+x;
            for(int i=0; i<num_threads; i++)
            {
                if(m_pipeline[i]->framebuffers()->z_buffer[index] < min_z)
                {
                    min_z = m_pipeline[i]->framebuffers()->z_buffer[index];
                    thread_id = i;
                }
            }
            (*m_framebuffers->color_buffer)(y, x, ColorBit::R) = (*m_pipeline[thread_id]->framebuffers()->color_buffer)(y, x, ColorBit::R);
            (*m_framebuffers->color_buffer)(y, x, ColorBit::G) = (*m_pipeline[thread_id]->framebuffers()->color_buffer)(y, x, ColorBit::G);
            (*m_framebuffers->color_buffer)(y, x, ColorBit::B) = (*m_pipeline[thread_id]->framebuffers()->color_buffer)(y, x, ColorBit::B);
            m_framebuffers->z_buffer[index] = min_z;
        }
}

}