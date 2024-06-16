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
    m_framebuffers->clear(BufferType::COLOR|BufferType::DEPTH);

    // two-pass rendering
    /* 1.first pass test depth from camera */
    auto cameraPos = ubo.cameraPos;
    int shadowMap_num = 0;
    for(int i=0; i<ubo.lights.size(); i++)
    {
        if(!ubo.lights[i].hasShadowMap)
            continue;
        #pragma omp parallel for num_threads(num_threads)
        for(int i=0; i<num_threads; i++)
            m_pipeline[i]->framebuffers()->clear(BufferType::DEPTH);
        // ubo.init(m_width, m_height, ubo.models, ubo.lights[i].pos, float(m_width)/float(m_height), Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 1.f, 0.f), 75.f/180.f*float(MY_PI), 0.1f, 1000.f, {0.f, -1.f, 1.f}, ubo.lights);
        ubo.cameraPos = ubo.lights[i].pos;
        ubo.shadowmap_zNear = 0.1f;
        ubo.shadowmap_zFar = 100.f;
        ubo.set_view(ubo.lights[i].pos, Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 1.f, 0.f));
        ubo.set_orthoProjection(-10.f, 10.f, -10.f, 10.f, ubo.shadowmap_zNear, ubo.shadowmap_zFar);
        // ubo.set_projection(75.f/180.f*float(MY_PI), float(m_width)/float(m_height), 0.1f, 100.f);
        ubo.lights[i].lightVP = ubo.projection * ubo.view;
        for(auto& obj : obj_loader.objects)
        {
            int use_threads = std::clamp(int(obj.indices.size())/3, 1, num_threads);

            for(int i=0; i<num_threads; i++)
            {
                m_pipeline[i]->set_state(point_VertexShader, depth_FragmentShader, obj.colorTexture, obj.normalTexture, std::min(m_width*m_height/int(obj.indices.size())*3*1000, m_width*m_height), PrimitiveType::TRIANGLE, ShadeFrequency::GOURAUD);
            }
            if(m_pipeline[0]->primitiveType()==PrimitiveType::TRIANGLE)
            {
                #pragma omp parallel for num_threads(use_threads)
                for(int i=0; i<obj.indices.size(); i+=3)
                {
                    int thread_id = omp_get_thread_num();
                    m_pipeline[thread_id]->update(obj.vertices[obj.indices[i]], obj.vertices[obj.indices[i+1]], obj.vertices[obj.indices[i+2]], obj.indices[i], obj.indices[i+1], obj.indices[i+2]);
                    m_pipeline[thread_id]->generate_shadowmap(obj.id, shadowMap_num);
                }
            }
            linkSubFramebuffers(BufferType::DEPTH, shadowMap_num);
        }
        
        ubo.lights[i].shadowMap = m_framebuffers->depth_buffer[shadowMap_num];
        shadowMap_num++;
    }
    /* 2.second pass from camera */
    #pragma omp parallel for num_threads(num_threads)
    for(int i=0; i<num_threads; i++)
        m_pipeline[i]->framebuffers()->clear(BufferType::COLOR);
    ubo.init(m_width, m_height, ubo.models, cameraPos, float(m_width)/float(m_height), Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 1.f, 0.f), 75.f/180.f*float(MY_PI), 0.1f, 100.f, {0.f, -1.f, 1.f}, ubo.lights);
    for(auto& obj : obj_loader.objects)
    {
        int use_threads = std::clamp(int(obj.indices.size())/3, 1, num_threads);

        for(int i=0; i<num_threads; i++)
        {
            m_pipeline[i]->set_state(obj.vertexShader, obj.fragmentShader, obj.colorTexture, obj.normalTexture, std::min(m_width*m_height/int(obj.indices.size())*3*1000, m_width*m_height), PrimitiveType::TRIANGLE, ShadeFrequency::GOURAUD);
        }

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
            #pragma omp parallel for num_threads(use_threads)
            for(int i=0; i<obj.indices.size(); i+=3)
            {
                int thread_id = omp_get_thread_num();
                m_pipeline[thread_id]->update(obj.vertices[obj.indices[i]], obj.vertices[obj.indices[i+1]], obj.vertices[obj.indices[i+2]], obj.indices[i], obj.indices[i+1], obj.indices[i+2]);
                m_pipeline[thread_id]->run(obj.id);
            }
        }
        linkSubFramebuffers(BufferType::COLOR);
    }

}

void Render::linkSubFramebuffers(BufferType type, int buf_id)
{
    Region region;
    for(int i=0; i<num_threads; i++)
        region = region.Union(m_pipeline[i]->renderRegion());

    if(type==BufferType::COLOR)
    {
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
    else if(type==BufferType::DEPTH)
    {
        #pragma omp parallel for num_threads(16)
        for(int x=region.min.x(); x<=region.max.x(); x++)
            for(int y=region.min.y(); y<=region.max.y(); y++)
            {
                int thread_id = 0;
                int index = y*m_width+x;
                float min_z = std::numeric_limits<float>::max();
                for(int i=0; i<num_threads; i++)
                {
                    if(m_pipeline[i]->framebuffers()->z_buffer[index] < min_z)
                    {
                        min_z = m_pipeline[i]->framebuffers()->z_buffer[index];
                        thread_id = i;
                    }
                }
                (*m_framebuffers->depth_buffer[buf_id])(y, x, ColorBit::B) = (*m_pipeline[thread_id]->framebuffers()->depth_buffer[buf_id])(y, x, ColorBit::B);
                (*m_framebuffers->depth_buffer[buf_id])(y, x, ColorBit::G) = (*m_pipeline[thread_id]->framebuffers()->depth_buffer[buf_id])(y, x, ColorBit::G);
                (*m_framebuffers->depth_buffer[buf_id])(y, x, ColorBit::R) = (*m_pipeline[thread_id]->framebuffers()->depth_buffer[buf_id])(y, x, ColorBit::R);
                (*m_framebuffers->depth_buffer[buf_id])(y, x, ColorBit::A) = (*m_pipeline[thread_id]->framebuffers()->depth_buffer[buf_id])(y, x, ColorBit::A);
            }
    }
}

}