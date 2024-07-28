#include "pipeline.hpp"


namespace openrenderer{

Pipeline::~Pipeline()
{
    m_framebuffers.reset();
    m_gbuffers.reset();
    clear();
}

void Pipeline::update(Vertex v)
{
    m_vertexs[0] = v;
}
void Pipeline::update(Vertex v0, Vertex v1, int idx0, int idx1)
{
    m_vertexs[0] = v0;
    m_indices[0] = idx0;
    m_vertexs[1] = v1;
    m_indices[1] = idx1;
}
void Pipeline::update(Vertex v0, Vertex v1, Vertex v2, int idx0, int idx1, int idx2)
{
    m_vertexs[0] = v0;
    m_indices[0] = idx0;
    m_vertexs[1] = v1;
    m_indices[1] = idx1;
    m_vertexs[2] = v2;
    m_indices[2] = idx2;
}

void Pipeline::clear()
{
    if(m_rasterPoints)
        delete []m_rasterPoints;
    m_rasterPoints = nullptr;
    m_colorTexture = nullptr;
    m_rasterSize = 0;
    m_maxRasterSize = 0;
    for(int i=0; i<3; i++)
    {
        m_vertexs[i] = Vertex();
        m_attrs[i] = vertex_shader_out();
        m_indices[i] = -1;
    }

}

void Pipeline::set_state(std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)> vertexShader, 
                            std::function<Eigen::Vector3f(const Point&)> fragmentShader, 
                            Texture* colorTexture, Texture* normalTexture, Gbuffers* defferedGbuffers,
                            int max_rasterSize, PrimitiveType primitive, ShadeFrequency freq)
{
    m_vertexShaderFunc = vertexShader;
    m_fragmentShaderFunc = fragmentShader;
    m_primitiveType = primitive;
    m_colorTexture = colorTexture;
    m_normalTexture = normalTexture;
    m_defferedGbuffers = defferedGbuffers;
    m_shadeFrequency = freq;
    m_rasterSize = 0;
    if(max_rasterSize > m_maxRasterSize)
    {
        if(m_rasterPoints)
            delete []m_rasterPoints;
        m_rasterPoints = new Point[max_rasterSize];
        m_maxRasterSize = max_rasterSize;
    }
}



void Pipeline::run(int obj_id)
{
    /*1.vertex shader*/
    Eigen::Vector3f tangent = Eigen::Vector3f::Zero();
    if(m_primitiveType==PrimitiveType::TRIANGLE) 
    {
        if(m_vertexs[0].normal==Eigen::Vector3f::Zero() || m_vertexs[1].normal==Eigen::Vector3f::Zero() || m_vertexs[2].normal==Eigen::Vector3f::Zero())
        {
            Eigen::Vector3f faceNormal = (m_vertexs[1].pos-m_vertexs[0].pos).cross(m_vertexs[2].pos-m_vertexs[0].pos).normalized();
            m_vertexs[0].normal = faceNormal;
            m_vertexs[1].normal = faceNormal;
            m_vertexs[2].normal = faceNormal;
        }
        tangent = calculate_tangent(m_vertexs);
    }
    for(int i=0; i<int(m_primitiveType); i++)
    {
        vertex_shader_out out_attr{Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero()};
        vertex_shader_in input{m_vertexs[i], obj_id, tangent, m_normalTexture};
        gl_Position[i] = vertex_shader(input, out_attr);
        // viewport transform
        int width = m_framebuffers->width, height = m_framebuffers->height;
        if(gl_Position[i].w()<0.f)  //remove out of the camera NDC-space triangle mesh
            return;
        // gl_Position[i].w() = std::abs(gl_Position[i].w()); 
        int x = int((gl_Position[i].x()/gl_Position[i].w()+1.f)*width/2.f);
        int y = int((gl_Position[i].y()/gl_Position[i].w()+1.f)*height/2.f);
        m_primitive[i].screen_pos = {x, y};
        m_primitive[i].z = gl_Position[i].z()/gl_Position[i].w();
        m_primitive[i].obj_id = input.obj_id;
        m_attrs[i] = out_attr;
    }
    /*2.primitive assembly*/
    primitive_assembly();
    /*3.rasterization*/
    rasterization();
    /*4.fragment shader*/
    #pragma omp parallel for if(m_rasterSize>10000) num_threads(7)
    for(int i=0; i<m_rasterSize; i++) 
    {
        if(test_depth(m_rasterPoints[i].screen_pos.x(), m_rasterPoints[i].screen_pos.y(), m_rasterPoints[i].z, 
                m_framebuffers->width, m_framebuffers->height, m_framebuffers->z_buffer))
        {
            m_rasterPoints[i].colorTexture = m_colorTexture;
            m_rasterPoints[i].normalTexture = m_normalTexture;
            m_rasterPoints[i].gbuffers = m_defferedGbuffers;
            Eigen::Vector3f color = fragment_shader(m_rasterPoints[i]);
            int x = m_rasterPoints[i].screen_pos.x();
            int y = m_rasterPoints[i].screen_pos.y();
            (*m_framebuffers->color_buffer)(y, x, ColorBit::R) = static_cast<uint8_t>(std::clamp(color.x(), 0.f, 1.f)*255.f);
            (*m_framebuffers->color_buffer)(y, x, ColorBit::G) = static_cast<uint8_t>(std::clamp(color.y(), 0.f, 1.f)*255.f);
            (*m_framebuffers->color_buffer)(y, x, ColorBit::B) = static_cast<uint8_t>(std::clamp(color.z(), 0.f, 1.f)*255.f);
        }
    }
}

void Pipeline::generate_gbuffers(int obj_id)
{
    Eigen::Vector3f tangent = Eigen::Vector3f::Zero();
    if(m_primitiveType==PrimitiveType::TRIANGLE) 
    {
        if(m_vertexs[0].normal==Eigen::Vector3f::Zero() || m_vertexs[1].normal==Eigen::Vector3f::Zero() || m_vertexs[2].normal==Eigen::Vector3f::Zero())
        {
            Eigen::Vector3f faceNormal = (m_vertexs[1].pos-m_vertexs[0].pos).cross(m_vertexs[2].pos-m_vertexs[0].pos).normalized();
            m_vertexs[0].normal = faceNormal;
            m_vertexs[1].normal = faceNormal;
            m_vertexs[2].normal = faceNormal;
        }
        tangent = calculate_tangent(m_vertexs);
    }
    for(int i=0; i<int(m_primitiveType); i++)
    {
        vertex_shader_out out_attr{Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero()};
        vertex_shader_in input{m_vertexs[i], obj_id, tangent};
        gl_Position[i] = vertex_shader(input, out_attr);
        // viewport transform
        int width = m_framebuffers->width, height = m_framebuffers->height;
        gl_Position[i].w() = std::abs(gl_Position[i].w()); 
        int x = int((gl_Position[i].x()/gl_Position[i].w()+1.f)*width/2.f);
        int y = int((gl_Position[i].y()/gl_Position[i].w()+1.f)*height/2.f);
        m_primitive[i].screen_pos = {x, y};
        m_primitive[i].z = gl_Position[i].z()/gl_Position[i].w();
        m_primitive[i].obj_id = input.obj_id;
        m_attrs[i] = out_attr;
    }
    /*2.primitive assembly*/
    primitive_assembly();
    /*3.rasterization*/
    Eigen::Vector2i box_min(4096, 2160); 
    Eigen::Vector2i box_max(0, 0);
    box_min.x() = std::clamp(std::min(m_primitive[0].screen_pos.x(), std::min(m_primitive[1].screen_pos.x(), m_primitive[2].screen_pos.x())), 0, m_framebuffers->width-1);
    box_min.y() = std::clamp(std::min(m_primitive[0].screen_pos.y(), std::min(m_primitive[1].screen_pos.y(), m_primitive[2].screen_pos.y())), 0, m_framebuffers->height-1);
    box_max.x() = std::clamp(std::max(m_primitive[0].screen_pos.x(), std::max(m_primitive[1].screen_pos.x(), m_primitive[2].screen_pos.x())), 0, m_framebuffers->width-1);
    box_max.y() = std::clamp(std::max(m_primitive[0].screen_pos.y(), std::max(m_primitive[1].screen_pos.y(), m_primitive[2].screen_pos.y())), 0, m_framebuffers->height-1);
    m_renderRegion = m_renderRegion.Union(Region{ box_min, box_max });
    int num = 0;
    for(int x=box_min.x(); x<=box_max.x(); x++)
    {
        for(int y=box_min.y(); y<=box_max.y(); y++)
        {
            Eigen::Vector3f bc = barycentric(m_primitive, Eigen::Vector2i(x, y));
            if(bc.x()<0.f || bc.y()<0.f || bc.z()<0.f)
                continue;

            float Z = correction(bc, gl_Position);
            m_rasterPoints[num].screen_pos = {x, y};
            m_rasterPoints[num].attrs.modelPos = bc.x()*m_primitive[0].attrs.modelPos + bc.y()*m_primitive[1].attrs.modelPos + bc.z()*m_primitive[2].attrs.modelPos;
            m_rasterPoints[num].z = Z;
            m_rasterPoints[num].v.pos.z() = (bc.x()*m_primitive[0].z + bc.y()*m_primitive[1].z + bc.z()*m_primitive[2].z);  // save the ndc-space depth
            m_rasterPoints[num].v.color = (bc.x()*m_primitive[0].v.color + bc.y()*m_primitive[1].v.color + bc.z()*m_primitive[2].v.color);
            m_rasterPoints[num].v.normal = (bc.x()*m_primitive[0].v.normal + bc.y()*m_primitive[1].v.normal + bc.z()*m_primitive[2].v.normal);
            m_rasterPoints[num].v.texCoord = (bc.x()*m_primitive[0].v.texCoord + bc.y()*m_primitive[1].v.texCoord + bc.z()*m_primitive[2].v.texCoord);
            m_rasterPoints[num].v.texCoord.x() = std::clamp(m_rasterPoints[num].v.texCoord.x(), 0.f, 1.f);
            m_rasterPoints[num].v.texCoord.y() = std::clamp(m_rasterPoints[num].v.texCoord.y(), 0.f, 1.f);
            m_rasterPoints[num].attrs.normal = (bc.x()*m_primitive[0].attrs.normal + bc.y()*m_primitive[1].attrs.normal + bc.z()*m_primitive[2].attrs.normal);
            m_rasterPoints[num].attrs.TBN = bc.x()*m_primitive[0].attrs.TBN + bc.y()*m_primitive[1].attrs.TBN + bc.z()*m_primitive[2].attrs.TBN;
            m_rasterPoints[num].attrs.ndcPos = bc.x()*m_primitive[0].attrs.ndcPos + bc.y()*m_primitive[1].attrs.ndcPos + bc.z()*m_primitive[2].attrs.ndcPos;
            num++;
            if(num > m_maxRasterSize) 
                break;
        }
        if(num > m_maxRasterSize)
            break;
    }
    m_rasterSize = num;
    /*4.fragment shader*/
    #pragma omp parallel for if(m_rasterSize>10000) num_threads(7)
    for(int i=0; i<m_rasterSize; i++) 
    {
        if(test_depth(m_rasterPoints[i].screen_pos.x(), m_rasterPoints[i].screen_pos.y(), m_rasterPoints[i].z, 
                m_framebuffers->width, m_framebuffers->height, m_framebuffers->z_buffer))
        {
            m_rasterPoints[i].colorTexture = m_colorTexture;
            m_rasterPoints[i].normalTexture = m_normalTexture;
            int x = m_rasterPoints[i].screen_pos.x();
            int y = m_rasterPoints[i].screen_pos.y();
            Eigen::Vector3f color = fragment_shader(m_rasterPoints[i]);
            Eigen::Vector4f viewPos = ubo.view * Eigen::Vector4f{m_rasterPoints[i].attrs.modelPos.x(), m_rasterPoints[i].attrs.modelPos.y(), m_rasterPoints[i].attrs.modelPos.z(), 1.f};
            (*m_gbuffers->position_buffer)(y, x, ColorBit::B) = m_rasterPoints[i].attrs.modelPos.x();
            (*m_gbuffers->position_buffer)(y, x, ColorBit::G) = m_rasterPoints[i].attrs.modelPos.y();
            (*m_gbuffers->position_buffer)(y, x, ColorBit::R) = m_rasterPoints[i].attrs.modelPos.z();
            (*m_gbuffers->position_buffer)(y, x, ColorBit::A) = viewPos.z();
            (*m_gbuffers->normal_buffer)(y, x, ColorBit::B) = m_rasterPoints[i].attrs.normal.x();
            (*m_gbuffers->normal_buffer)(y, x, ColorBit::G) = m_rasterPoints[i].attrs.normal.y();
            (*m_gbuffers->normal_buffer)(y, x, ColorBit::R) = m_rasterPoints[i].attrs.normal.z();
            (*m_gbuffers->albedo_buffer)(y, x, ColorBit::B) = color.x();
            (*m_gbuffers->albedo_buffer)(y, x, ColorBit::G) = color.y();
            (*m_gbuffers->albedo_buffer)(y, x, ColorBit::R) = color.z();
        }
    }
}

void Pipeline::generate_shadowmap(int obj_id, int shadowmap_id)
{
    for(int i=0; i<int(m_primitiveType); i++)
    {
        vertex_shader_out out_attr{Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero()};
        vertex_shader_in input{m_vertexs[i], obj_id};
        gl_Position[i] = vertex_shader(input, out_attr);
        // viewport transform
        int width = m_framebuffers->width, height = m_framebuffers->height;
        gl_Position[i].w() = std::abs(gl_Position[i].w()); 
        int x = int((gl_Position[i].x()/gl_Position[i].w()+1.f)*width/2.f);
        int y = int((gl_Position[i].y()/gl_Position[i].w()+1.f)*height/2.f);
        m_primitive[i].screen_pos = {x, y};
        m_primitive[i].z = gl_Position[i].z()/gl_Position[i].w();
        m_primitive[i].obj_id = input.obj_id;
        m_attrs[i] = out_attr;
    }
    /*2.primitive assembly*/
    primitive_assembly();
    /*3.rasterization*/
    Eigen::Vector2i box_min(4096, 2160); 
    Eigen::Vector2i box_max(0, 0);
    box_min.x() = std::clamp(std::min(m_primitive[0].screen_pos.x(), std::min(m_primitive[1].screen_pos.x(), m_primitive[2].screen_pos.x())), 0, m_framebuffers->width-1);
    box_min.y() = std::clamp(std::min(m_primitive[0].screen_pos.y(), std::min(m_primitive[1].screen_pos.y(), m_primitive[2].screen_pos.y())), 0, m_framebuffers->height-1);
    box_max.x() = std::clamp(std::max(m_primitive[0].screen_pos.x(), std::max(m_primitive[1].screen_pos.x(), m_primitive[2].screen_pos.x())), 0, m_framebuffers->width-1);
    box_max.y() = std::clamp(std::max(m_primitive[0].screen_pos.y(), std::max(m_primitive[1].screen_pos.y(), m_primitive[2].screen_pos.y())), 0, m_framebuffers->height-1);
    m_renderRegion = m_renderRegion.Union(Region{ box_min, box_max });
    int num = 0;
    for(int x=box_min.x(); x<=box_max.x(); x++)
    {
        for(int y=box_min.y(); y<=box_max.y(); y++)
        {
            Eigen::Vector3f bc = barycentric(m_primitive, Eigen::Vector2i(x, y));
            if(bc.x()<0.f || bc.y()<0.f || bc.z()<0.f)
                continue;

            float Z = correction(bc, gl_Position);
            m_rasterPoints[num].screen_pos = {x, y};
            m_rasterPoints[num].attrs.ndcPos = bc.x()*m_primitive[0].attrs.ndcPos + bc.y()*m_primitive[1].attrs.ndcPos + bc.z()*m_primitive[2].attrs.ndcPos;
            num++;
            if(num > m_maxRasterSize) 
                break;
        }
        if(num > m_maxRasterSize)
            break;
    }
    m_rasterSize = num;
    /*4.fragment shader*/
    #pragma omp parallel for if(m_rasterSize>10000) num_threads(3)
    for(int i=0; i<m_rasterSize; i++) 
    {
        int index = m_rasterPoints[i].screen_pos.y() * m_framebuffers->width + m_rasterPoints[i].screen_pos.x();
        if(m_rasterPoints[i].attrs.ndcPos.z()>0.f && m_rasterPoints[i].attrs.ndcPos.z() < m_framebuffers->z_buffer[index]-1e-4)
        {
            m_framebuffers->z_buffer[index] = m_rasterPoints[i].attrs.ndcPos.z();
            int x = m_rasterPoints[i].screen_pos.x();
            int y = m_rasterPoints[i].screen_pos.y();
            Float32ToUint8 value;
            value.num = m_rasterPoints[i].attrs.ndcPos.z();
            (*m_framebuffers->depth_buffer[shadowmap_id])(y, x, ColorBit::B) = value.arr[0];
            (*m_framebuffers->depth_buffer[shadowmap_id])(y, x, ColorBit::G) = value.arr[1];
            (*m_framebuffers->depth_buffer[shadowmap_id])(y, x, ColorBit::R) = value.arr[2];
            (*m_framebuffers->depth_buffer[shadowmap_id])(y, x, ColorBit::A) = value.arr[3];
        }
    }
}

    
Eigen::Vector4f Pipeline::vertex_shader(const vertex_shader_in& intput, vertex_shader_out& output)
{
    return m_vertexShaderFunc(intput, output);
}

void Pipeline::primitive_assembly()
{
    for(int i=0; i<int(m_primitiveType); i++)
    {
        m_primitive[i].v = m_vertexs[i];
        m_primitive[i].colorTexture = m_colorTexture;
        m_primitive[i].normalTexture = m_normalTexture;
        m_primitive[i].gbuffers = m_defferedGbuffers;
        m_primitive[i].attrs = m_attrs[i];
    }
}

void Pipeline::rasterization()
{
    auto start = std::chrono::high_resolution_clock::now();
    if(m_primitiveType==PrimitiveType::POINT)
    {
        if(m_primitive[0].screen_pos.x()<0||m_primitive[0].screen_pos.x()>m_framebuffers->width-1 || m_primitive[0].screen_pos.y()<0||m_primitive[0].screen_pos.y()>m_framebuffers->height-1)
            m_rasterSize = 0;
        else
        {
            m_rasterSize = 1;
            m_rasterPoints[0] = m_primitive[0];

            Eigen::Vector2i box_min(4096, 2160); 
            Eigen::Vector2i box_max(0, 0);
            box_min.x() = std::clamp(m_rasterPoints[0].screen_pos.x()-1, 0, m_framebuffers->width-1);
            box_min.y() = std::clamp(m_rasterPoints[0].screen_pos.y()-1, 0, m_framebuffers->height-1);
            box_max.x() = std::clamp(m_rasterPoints[0].screen_pos.x()+1, 0, m_framebuffers->width-1);
            box_max.y() = std::clamp(m_rasterPoints[0].screen_pos.y()+1, 0, m_framebuffers->height-1);
            m_renderRegion = m_renderRegion.Union(Region{ box_min, box_max });
        }
    }
    else if(m_primitiveType==PrimitiveType::LINE)
        m_rasterSize = line(m_primitive, m_rasterPoints, m_maxRasterSize, m_framebuffers->width, m_framebuffers->height, m_renderRegion);
    else if(m_primitiveType==PrimitiveType::TRIANGLE)
        m_rasterSize = triangle(m_primitive, gl_Position, m_rasterPoints, m_maxRasterSize, m_shadeFrequency, m_framebuffers->width, m_framebuffers->height, m_renderRegion);
    auto end = std::chrono::high_resolution_clock::now();
    t4 += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-6;
}

Eigen::Vector3f Pipeline::fragment_shader(const Point& point_input)
{
    return m_fragmentShaderFunc(point_input);
}


int line(const Point* input, Point* raster_region, int max_size, int width, int height, Region& renderRegion)
{
    Eigen::Vector2i box_min(4096, 2160); 
    Eigen::Vector2i box_max(0, 0);
    box_min.x() = std::clamp(std::min(input[0].screen_pos.x(), input[1].screen_pos.x()), 0, width-1);
    box_min.y() = std::clamp(std::min(input[0].screen_pos.y(), input[1].screen_pos.y()), 0, height-1);
    box_max.x() = std::clamp(std::max(input[0].screen_pos.x(), input[1].screen_pos.x()), 0, width-1);
    box_max.y() = std::clamp(std::max(input[0].screen_pos.y(), input[1].screen_pos.y()), 0, height-1);
    renderRegion = renderRegion.Union(Region{ box_min, box_max });
    bool is_steep = false;
    int x0 = input[0].screen_pos.x(), y0 = input[0].screen_pos.y();
    int x1 = input[1].screen_pos.x(), y1 = input[1].screen_pos.y();
    if(std::abs(x1-x0)<std::abs(y1-y0))
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
        is_steep = true;
    }
    if(x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    const int dx = x1 - x0;
    const int abs_k = std::abs(y1-y0) * 2;
    int error = 0;
    int y = y0;
    const int step = y0<y1 ? 1 : -1; 
    int num = 0;
    Eigen::Vector2f bc;
    for(int x=x0; x<=x1 && num<max_size; x++)
    {
        if(is_steep)
        {
            raster_region[num].screen_pos = {y, x};
            raster_region[num].obj_id = input->obj_id;
        }
        else
        {
            raster_region[num].screen_pos = {x, y};
            raster_region[num].obj_id = input->obj_id;
        }
        error += abs_k;
        if(error > dx)
        {
            y += step;
            error -= dx*2;
        }
        if(raster_region[num].screen_pos.x()<0||raster_region[num].screen_pos.x()>width-1 || raster_region[num].screen_pos.y()<0||raster_region[num].screen_pos.y()>height-1)
            continue;
        num++;
    }
    return num;
}
Eigen::Vector2f barycentric_line(const Eigen::Vector2i& v0, const Eigen::Vector2i& v1, const Eigen::Vector2i& p)
{
    float k = (p.x()-v0.x()) / float(v1.x()-v0.x());
    return { 1.f-k, k };
}

Eigen::Vector3f barycentric(const Point *vs, const Eigen::Vector2i& p) 
{ 
    // float c1 = (p.x()*(vs[1].screen_pos.y() - vs[2].screen_pos.y()) + (vs[2].screen_pos.x() - vs[1].screen_pos.x())*p.y() + vs[1].screen_pos.x()*vs[2].screen_pos.y() - vs[2].screen_pos.x()*vs[1].screen_pos.y()) / (vs[0].screen_pos.x()*(vs[1].screen_pos.y() - vs[2].screen_pos.y()) + (vs[2].screen_pos.x() - vs[1].screen_pos.x())*vs[0].screen_pos.y() + vs[1].screen_pos.x()*vs[2].screen_pos.y() - vs[2].screen_pos.x()*vs[1].screen_pos.y());
    // float c2 = (p.x()*(vs[2].screen_pos.y() - vs[0].screen_pos.y()) + (vs[0].screen_pos.x() - vs[2].screen_pos.x())*p.y() + vs[2].screen_pos.x()*vs[0].screen_pos.y() - vs[0].screen_pos.x()*vs[2].screen_pos.y()) / (vs[1].screen_pos.x()*(vs[2].screen_pos.y() - vs[0].screen_pos.y()) + (vs[0].screen_pos.x() - vs[2].screen_pos.x())*vs[1].screen_pos.y() + vs[2].screen_pos.x()*vs[0].screen_pos.y() - vs[0].screen_pos.x()*vs[2].screen_pos.y());
    // float c3 = (p.x()*(vs[0].screen_pos.y() - vs[1].screen_pos.y()) + (vs[1].screen_pos.x() - vs[0].screen_pos.x())*p.y() + vs[0].screen_pos.x()*vs[1].screen_pos.y() - vs[1].screen_pos.x()*vs[0].screen_pos.y()) / (vs[2].screen_pos.x()*(vs[0].screen_pos.y() - vs[1].screen_pos.y()) + (vs[1].screen_pos.x() - vs[0].screen_pos.x())*vs[2].screen_pos.y() + vs[0].screen_pos.x()*vs[1].screen_pos.y() - vs[1].screen_pos.x()*vs[0].screen_pos.y());
    // return Eigen::Vector3f(c1, c2, c3);

    Eigen::Vector3i u = Eigen::Vector3i(vs[2].screen_pos.x()-vs[0].screen_pos.x(), vs[1].screen_pos.x()-vs[0].screen_pos.x(), vs[0].screen_pos.x()-p.x()).cross(Eigen::Vector3i(vs[2].screen_pos.y()-vs[0].screen_pos.y(), vs[1].screen_pos.y()-vs[0].screen_pos.y(), vs[0].screen_pos.y()-p.y()));
    if (std::abs(u.z())<1) return Eigen::Vector3f(-1.f,1.f,1.f);
    return Eigen::Vector3f(1.f-(u.x()+u.y())/float(u.z()), u.y()/float(u.z()), u.x()/float(u.z())); 
}

float correction(Eigen::Vector3f& bc, const Eigen::Vector4f* gl_Position)
{
    // alphe = (1-u-v)/(Z1-c), beta = u/(Z2-c), gamma = v/(Z3-c)
    float alphe = bc.x() / gl_Position[0].w();
    float beta = bc.y() / gl_Position[1].w();
    float gamma = bc.z() / gl_Position[2].w();
    float Z = 1.f / (alphe+beta+gamma);
    bc.x() = alphe * Z;
    bc.y() = beta * Z;
    bc.z() = gamma * Z;
    return Z;
}

bool test_depth(int x, int y, float z, int width, int height, float* z_buffer)
{
    int index = y * width + x;
    bool is_shade = false;
    if(z < z_buffer[index] - 1e-4)
    {
        z_buffer[index] = z;
        is_shade = true;
    }
    return is_shade;
}

int triangle(const Point* input, const Eigen::Vector4f* gl_Position, Point* raster_region, int max_size, ShadeFrequency freq, int width, int height, Region& renderRegion)
{
    Eigen::Vector2i box_min(4096, 2160); 
    Eigen::Vector2i box_max(0, 0);
    box_min.x() = std::clamp(std::min(input[0].screen_pos.x(), std::min(input[1].screen_pos.x(), input[2].screen_pos.x())), 0, width-1);
    box_min.y() = std::clamp(std::min(input[0].screen_pos.y(), std::min(input[1].screen_pos.y(), input[2].screen_pos.y())), 0, height-1);
    box_max.x() = std::clamp(std::max(input[0].screen_pos.x(), std::max(input[1].screen_pos.x(), input[2].screen_pos.x())), 0, width-1);
    box_max.y() = std::clamp(std::max(input[0].screen_pos.y(), std::max(input[1].screen_pos.y(), input[2].screen_pos.y())), 0, height-1);
    renderRegion = renderRegion.Union(Region{ box_min, box_max });
    int num = 0;

    if(freq==ShadeFrequency::FLAT)
    {
        // float faceZ = 0.333f*(input[0].z+input[1].z+input[2].z);
        Eigen::Vector3f faceNormal = (input[1].v.pos-input[0].v.pos).cross(input[2].v.pos-input[0].v.pos);
        Eigen::Vector3f afterFaceNormal = (input[0].attrs.normal+input[1].attrs.normal+input[2].attrs.normal);
        Eigen::Vector2f faceTexCoord = 0.333f*(input[0].v.texCoord+input[1].v.texCoord+input[2].v.texCoord);
        Eigen::Matrix3f faceTBN = 0.333f*(input[0].attrs.TBN+input[1].attrs.TBN+input[2].attrs.TBN);
        Eigen::Vector3f faceNDCPosition = 0.333f*(input[0].attrs.ndcPos+input[1].attrs.ndcPos+input[2].attrs.ndcPos);
        Eigen::Vector3f faceModelPosition = 0.333f*(input[0].attrs.modelPos+input[1].attrs.modelPos+input[2].attrs.modelPos);
        Eigen::Vector3f faceWorldPosition = 0.333f*(input[0].v.pos+input[1].v.pos+input[2].v.pos);
        Eigen::Vector3f faceColor = 0.333f*(input[0].v.color+input[1].v.color+input[2].v.color);
        for(int x=box_min.x(); x<=box_max.x(); x++)
            for(int y=box_min.y(); y<=box_max.y(); y++)
            {
                Eigen::Vector3f bc = barycentric(input, Eigen::Vector2i(x, y));
                if(bc.x()<0 || bc.y()<0 || bc.z()<0)
                    continue;
                float Z = correction(bc, gl_Position);
                raster_region[num].obj_id = input[0].obj_id;
                raster_region[num].screen_pos = {x, y};
                raster_region[num].z = Z;
                raster_region[num].v.pos = faceWorldPosition;
                raster_region[num].v.color = faceColor;
                raster_region[num].v.normal = faceNormal;
                raster_region[num].v.texCoord = faceTexCoord;
                raster_region[num].attrs.normal = afterFaceNormal;
                raster_region[num].attrs.TBN = faceTBN;
                raster_region[num].attrs.ndcPos = faceNDCPosition;
                raster_region[num].attrs.modelPos = faceModelPosition;
                num++; 
                if(num > max_size) 
                {
                    return num;
                }
            }
    }
    else if(freq==ShadeFrequency::GOURAUD)
    {
        for(int x=box_min.x(); x<=box_max.x(); x++)
           for(int y=box_min.y(); y<=box_max.y(); y++)
           {
                Eigen::Vector3f bc = barycentric(input, Eigen::Vector2i(x, y));
                if(bc.x()<0.f || bc.y()<0.f || bc.z()<0.f)
                    continue;

                float Z = correction(bc, gl_Position);
                raster_region[num].obj_id = input[0].obj_id;
                raster_region[num].screen_pos = {x, y};
                raster_region[num].z = Z;
                raster_region[num].v.pos = (bc.x()*input[0].v.pos + bc.y()*input[1].v.pos + bc.z()*input[2].v.pos);
                raster_region[num].v.color = (bc.x()*input[0].v.color + bc.y()*input[1].v.color + bc.z()*input[2].v.color);
                raster_region[num].v.normal = (bc.x()*input[0].v.normal + bc.y()*input[1].v.normal + bc.z()*input[2].v.normal);
                raster_region[num].v.texCoord = (bc.x()*input[0].v.texCoord + bc.y()*input[1].v.texCoord + bc.z()*input[2].v.texCoord);
                raster_region[num].v.texCoord.x() = std::clamp(raster_region[num].v.texCoord.x(), 0.f, 1.f);
                raster_region[num].v.texCoord.y() = std::clamp(raster_region[num].v.texCoord.y(), 0.f, 1.f);
                raster_region[num].attrs.normal = (bc.x()*input[0].attrs.normal + bc.y()*input[1].attrs.normal + bc.z()*input[2].attrs.normal);
                raster_region[num].attrs.TBN = bc.x()*input[0].attrs.TBN + bc.y()*input[1].attrs.TBN + bc.z()*input[2].attrs.TBN;
                raster_region[num].attrs.ndcPos = bc.x()*input[0].attrs.ndcPos + bc.y()*input[1].attrs.ndcPos + bc.z()*input[2].attrs.ndcPos;
                raster_region[num].attrs.modelPos = bc.x()*input[0].attrs.modelPos + bc.y()*input[1].attrs.modelPos + bc.z()*input[2].attrs.modelPos;
                num++;
                if(num > max_size) 
                {
                    return num;
                }
           }
    }
    return num;
}

int thread_schedule(int thread_id, int i, int total_threads)
{
    return thread_id + i*total_threads;
}

Eigen::Vector3f calculate_tangent(const Vertex points[3])
{
    if(points[0].texCoord==Eigen::Vector2f::Zero()&&points[1].texCoord==Eigen::Vector2f::Zero()&&points[2].texCoord==Eigen::Vector2f::Zero())
    {
        std::uniform_real_distribution<float> randomFloats(0.f, 1.f);
        std::default_random_engine generator;
        return Eigen::Vector3f{randomFloats(generator)*2.0f-1.0f, randomFloats(generator)*2.0f-1.0f, 0.f}.normalized();
    }
    Eigen::Vector3f tangent;
    Eigen::Vector3f edge1 = points[1].pos - points[0].pos;
    Eigen::Vector3f edge2 = points[2].pos - points[0].pos;
    Eigen::Vector2f dUV1 = points[1].texCoord - points[0].texCoord;
    Eigen::Vector2f dUV2 = points[2].texCoord - points[0].texCoord;

    float f = 1.0f / (dUV1.x() * dUV2.y() - dUV2.x() * dUV1.y());
    tangent.x() = f * (dUV2.y() * edge1.x() - dUV1.y() * edge2.x());
    tangent.y() = f * (dUV2.y() * edge1.y() - dUV1.y() * edge2.y());
    tangent.z() = f * (dUV2.y() * edge1.z() - dUV1.y() * edge2.z());

    return tangent.normalized();
}

}