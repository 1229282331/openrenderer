#include "renderer.hpp"



namespace openrenderer{

Render::Render(int w, int h, bool enable_color, bool enable_depth, bool enable_defferedRender, PixelFormat color_format, PixelFormat depth_format, PixelFormat gbuffer_format) : m_width(w), m_height(h), m_isDefferedRender(enable_defferedRender)
{
    m_framebuffers = std::make_unique<Framebuffers>(w, h, enable_color, enable_depth, color_format, depth_format);
    if(enable_defferedRender)
        m_gbuffers = std::make_unique<Gbuffers>(w, h, gbuffer_format);
    m_isDefferedRender = enable_defferedRender;
}

Render::~Render()
{
    m_framebuffers.reset();
    m_gbuffers.reset();
    for(int i=0; i<num_threads; i++)
        m_pipeline[i].reset();
}

void Render::init_pipeline(PrimitiveType primitive, ShadeFrequency freq, 
                            std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)> vertexShaderFunc, 
                            std::function<Eigen::Vector3f(const Point&)> fragmentShaderFunc)
{
    for(int i=0; i<num_threads; i++)
        m_pipeline[i] = std::make_unique<Pipeline>(primitive, freq, vertexShaderFunc, fragmentShaderFunc, *m_framebuffers, m_gbuffers.get());
}


void Render::drawFrame(const Loader& obj_loader)
{
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    if(!m_pipeline)
    {
        printf("[error] pipeline hasn't set!\n");
        return;
    }
    for(int i=0; i<num_threads; i++)
        m_pipeline[i]->renderRegion().reset();
    m_framebuffers->clear(BufferType::COLOR|BufferType::DEPTH);
    if(m_isDefferedRender)
        m_gbuffers->clear();

    // two-pass rendering
    /* 1.first pass test depth from camera */
    auto cameraPos = ubo.cameraPos;
    int shadowMap_num = 0;
    int firstPass_pipelineNum = std::min(2, num_threads);
    for(int i=0; i<ubo.lights.size(); i++)
    {
        if(!ubo.lights[i].hasShadowMap)
            continue;
        start = std::chrono::high_resolution_clock::now();
        for(int i=0; i<firstPass_pipelineNum; i++)
            m_pipeline[i]->framebuffers()->clear(BufferType::DEPTH);
        end = std::chrono::high_resolution_clock::now();
        t3 += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-6;

        // ubo.init(m_width, m_height, ubo.models, ubo.lights[i].pos, float(m_width)/float(m_height), Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 1.f, 0.f), 75.f/180.f*float(MY_PI), 0.1f, 1000.f, {0.f, -1.f, 1.f}, ubo.lights);
        ubo.cameraPos = ubo.lights[i].pos;
        ubo.shadowmap_zNear = 0.1f;
        ubo.shadowmap_zFar = 100.f;
        ubo.set_view(ubo.lights[i].pos, Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 1.f, 0.f));
        ubo.set_orthoProjection(-10.f, 10.f, -10.f, 10.f, ubo.shadowmap_zNear, ubo.shadowmap_zFar);
        // ubo.set_projection(75.f/180.f*float(MY_PI), float(m_width)/float(m_height), 0.1f, 100.f);
        ubo.lights[i].lightVP = ubo.projection * ubo.view;
        start = std::chrono::high_resolution_clock::now();
        for(auto& obj : obj_loader.objects)
        {
            int use_threads = std::clamp(int(obj.indices.size())/3, 1, firstPass_pipelineNum);

            for(int i=0; i<firstPass_pipelineNum; i++)
            {
                m_pipeline[i]->set_state(point_VertexShader, depth_FragmentShader, obj.colorTexture, obj.normalTexture, nullptr, std::min(m_width*m_height/int(obj.indices.size())*3*1000, m_width*m_height), PrimitiveType::TRIANGLE, ShadeFrequency::GOURAUD);
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
        }
        end = std::chrono::high_resolution_clock::now();
        t0 += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-6;
        linkSubFramebuffers(firstPass_pipelineNum, BufferType::DEPTH, shadowMap_num);
        ubo.lights[i].shadowMap = m_framebuffers->depth_buffer[shadowMap_num];
        shadowMap_num++;
    }
    /* 2.second pass from camera */
    int defferedPass_pipelineNum = std::min(2, num_threads);
    ubo.init(m_width, m_height, ubo.models, cameraPos, float(m_width)/float(m_height), Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 1.f, 0.f), 75.f/180.f*float(MY_PI), 0.1f, 100.f, {0.f, -1.f, 1.f}, ubo.lights);
    /* deffered rendering */
    if(m_isDefferedRender)
    {
        start = std::chrono::high_resolution_clock::now();
        for(int i=0; i<defferedPass_pipelineNum; i++)
        {
            m_pipeline[i]->framebuffers()->clearZ();
            m_pipeline[i]->gbuffers()->position_buffer->clear(0xf1);
        }
        end = std::chrono::high_resolution_clock::now();
        t3 += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-6;
        start = std::chrono::high_resolution_clock::now();
        for(auto& obj : obj_loader.objects)
        {
            int use_threads = std::clamp(int(obj.indices.size())/3, 1, defferedPass_pipelineNum);
            for(int i=0; i<defferedPass_pipelineNum; i++)
            {
                m_pipeline[i]->set_state(gbuffer_VertexShader, texture_FragmentShader, obj.colorTexture, obj.normalTexture, nullptr, std::min(m_width*m_height/int(obj.indices.size())*3*1000, m_width*m_height), PrimitiveType::TRIANGLE, ShadeFrequency::GOURAUD);
            }
            #pragma omp parallel for num_threads(use_threads)
            for(int i=0; i<obj.indices.size(); i+=3)
            {
                int thread_id = omp_get_thread_num();
                m_pipeline[thread_id]->update(obj.vertices[obj.indices[i]], obj.vertices[obj.indices[i+1]], obj.vertices[obj.indices[i+2]], obj.indices[i], obj.indices[i+1], obj.indices[i+2]);
                m_pipeline[thread_id]->generate_gbuffers(obj.id);
            }
        }
        end = std::chrono::high_resolution_clock::now();
        t1 += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-6;
        linkSubGbuffers(defferedPass_pipelineNum);
        ubo.depth_mipmap = new MipMap<float, 8>(*m_gbuffers->position_buffer);
    }
    /* simple rendering */
    start = std::chrono::high_resolution_clock::now();
    for(int i=0; i<num_threads; i++)
        m_pipeline[i]->framebuffers()->clear(BufferType::COLOR);
    end = std::chrono::high_resolution_clock::now();
    t3 += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-6;

    start = std::chrono::high_resolution_clock::now();
    for(auto& obj : obj_loader.objects)
    {
        int use_threads = std::clamp(int(obj.indices.size())/3, 1, num_threads);

        for(int i=0; i<num_threads; i++)
            m_pipeline[i]->set_state(obj.vertexShader, obj.fragmentShader, obj.colorTexture, obj.normalTexture, m_gbuffers.get(), std::min(m_width*m_height/int(obj.indices.size())*3*1000, m_width*m_height), PrimitiveType::TRIANGLE, ShadeFrequency::GOURAUD);

        if(m_pipeline[0]->primitiveType()==PrimitiveType::POINT)
        {
            #pragma omp parallel for num_threads(use_threads)
            for(int i=0; i<obj.vertices.size(); i++)
            {
                int thread_id = omp_get_thread_num();
                m_pipeline[thread_id]->update(obj.vertices[i]);
                m_pipeline[thread_id]->run(obj.id);
            }
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
    }
    end = std::chrono::high_resolution_clock::now();
    t2 += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-6;
    linkSubFramebuffers(num_threads, BufferType::COLOR);
}

void Render::linkSubFramebuffers(int use_pipelineNum, BufferType type, int buf_id)
{
    Region region;
    for(int i=0; i<use_pipelineNum; i++)
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
                for(int i=0; i<use_pipelineNum; i++)
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
                for(int i=0; i<use_pipelineNum; i++)
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

void Render::linkSubGbuffers(int use_pipelineNum)
{
    Region region;
    for(int i=0; i<use_pipelineNum; i++)
        region = region.Union(m_pipeline[i]->renderRegion());

    #pragma omp parallel for num_threads(16)
    for(int x=region.min.x(); x<=region.max.x(); x++)
        for(int y=region.min.y(); y<=region.max.y(); y++)
        {
            float min_z = std::numeric_limits<float>::max();
            int thread_id = 0;
            int index = y*m_width+x;
            for(int i=0; i<use_pipelineNum; i++)
            {
                if(m_pipeline[i]->framebuffers()->z_buffer[index] < min_z)
                {
                    min_z = m_pipeline[i]->framebuffers()->z_buffer[index];
                    thread_id = i;
                }
            }
            (*m_gbuffers->position_buffer)(y, x, ColorBit::B) = (*m_pipeline[thread_id]->gbuffers()->position_buffer)(y, x, ColorBit::B);
            (*m_gbuffers->position_buffer)(y, x, ColorBit::G) = (*m_pipeline[thread_id]->gbuffers()->position_buffer)(y, x, ColorBit::G);
            (*m_gbuffers->position_buffer)(y, x, ColorBit::R) = (*m_pipeline[thread_id]->gbuffers()->position_buffer)(y, x, ColorBit::R);
            (*m_gbuffers->position_buffer)(y, x, ColorBit::A) = (*m_pipeline[thread_id]->gbuffers()->position_buffer)(y, x, ColorBit::A);
            (*m_gbuffers->normal_buffer)(y, x, ColorBit::B) = (*m_pipeline[thread_id]->gbuffers()->normal_buffer)(y, x, ColorBit::B);
            (*m_gbuffers->normal_buffer)(y, x, ColorBit::G) = (*m_pipeline[thread_id]->gbuffers()->normal_buffer)(y, x, ColorBit::G);
            (*m_gbuffers->normal_buffer)(y, x, ColorBit::R) = (*m_pipeline[thread_id]->gbuffers()->normal_buffer)(y, x, ColorBit::R);
            (*m_gbuffers->albedo_buffer)(y, x, ColorBit::B) = (*m_pipeline[thread_id]->gbuffers()->albedo_buffer)(y, x, ColorBit::B);
            (*m_gbuffers->albedo_buffer)(y, x, ColorBit::G) = (*m_pipeline[thread_id]->gbuffers()->albedo_buffer)(y, x, ColorBit::G);
            (*m_gbuffers->albedo_buffer)(y, x, ColorBit::R) = (*m_pipeline[thread_id]->gbuffers()->albedo_buffer)(y, x, ColorBit::R);
            m_framebuffers->z_buffer[index] = min_z;
        }
    
}

}