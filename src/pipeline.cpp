#include "pipeline.hpp"


namespace openrenderer{

Pipeline::~Pipeline()
{
    m_framebuffers = nullptr;
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
    if(!m_rasterPoints)
        delete []m_rasterPoints;
    m_rasterPoints = nullptr;
    m_rasterSize = 0;
    m_maxRasterSize = 0;
    for(int i=0; i<3; i++)
    {
        m_vertexs[i] = Vertex();
        m_attrs[i] = vertex_shader_out();
        m_indices[i] = -1;
    }

}

void Pipeline::set_state(std::function<Eigen::Vector3f(const vertex_shader_in&, vertex_shader_out&)> vertexShader, 
                            std::function<Eigen::Vector3f(const Point&)> fragmentShader, 
                            int max_rasterSize, PrimitiveType primitive, ShadeFrequency freq)
{
    m_vertexShaderFunc = vertexShader;
    m_fragmentShaderFunc = fragmentShader;
    m_primitiveType = primitive;
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
    for(int i=0; i<int(m_primitiveType); i++)
    {
        vertex_shader_out out_attr{};
        vertex_shader_in input{m_vertexs[i], obj_id};
        Eigen::Vector3f gl_Position = vertex_shader(input, out_attr);
        // viewport transform
        int width = m_framebuffers->width, height = m_framebuffers->height;
        int x = std::clamp(int((gl_Position.x()+1.f)*width/2), 0, width-1);
        int y = std::clamp(int((gl_Position.y()+1.f)*height/2), 0, height-1);
        m_primitve[i].z = gl_Position.z();
        out_attr.screenPos << x, y;
        m_attrs[i] = out_attr;
    }
    /*2.primitive assembly*/
    primitive_assembly();
    /*3.rasterization*/
    rasterization();
    /*4.fragment shader*/
// #pragma omp parallel for
    for(int i=0; i<m_rasterSize; i++)
    {
        if(!test_depth(m_rasterPoints[i].screen_pos.x(), m_rasterPoints[i].screen_pos.y(), m_rasterPoints[i].z, 
                m_framebuffers->width, m_framebuffers->height, m_framebuffers->z_buffer))
            continue;
        Eigen::Vector3f color = fragment_shader(m_rasterPoints[i]);
        int x = m_rasterPoints[i].screen_pos.x();
        int y = m_rasterPoints[i].screen_pos.y();
        (*m_framebuffers->color_buffer)(y, x, ColorBit::R) = static_cast<uint8_t>(color.x()*255);
        (*m_framebuffers->color_buffer)(y, x, ColorBit::G) = static_cast<uint8_t>(color.y()*255);
        (*m_framebuffers->color_buffer)(y, x, ColorBit::B) = static_cast<uint8_t>(color.z()*255);
    }
    
}
    
Eigen::Vector3f Pipeline::vertex_shader(const vertex_shader_in& intput, vertex_shader_out& output)
{
    return m_vertexShaderFunc(intput, output);
}

void Pipeline::primitive_assembly()
{
    for(int i=0; i<int(m_primitiveType); i++)
    {
        m_primitve[i].v = m_vertexs[i];
        m_primitve[i].attrs = m_attrs[i];
        m_primitve[i].screen_pos = m_attrs[i].screenPos;
    }
}

void Pipeline::rasterization()
{
    if(m_primitiveType==PrimitiveType::POINT)
    {
        m_rasterSize = 1;
        m_rasterPoints[0] = m_primitve[0];
    }
    else if(m_primitiveType==PrimitiveType::LINE)
        m_rasterSize = line(m_primitve, m_rasterPoints, m_maxRasterSize);
    else if(m_primitiveType==PrimitiveType::TRIANGLE)
        m_rasterSize = triangle(m_primitve, m_rasterPoints, m_maxRasterSize, m_shadeFrequency);
}

Eigen::Vector3f Pipeline::fragment_shader(const Point& point_input)
{
    return m_fragmentShaderFunc(point_input);
}


int line(const Point* input, Point* raster_region, int max_size)
{
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
    for(int x=x0; x<=x1 && num<max_size; x++)
    {
        if(is_steep)
            raster_region[num].screen_pos = {y, x};
        else
            raster_region[num].screen_pos = {x, y};
        num++;
        error += abs_k;
        if(error > dx)
        {
            y += step;
            error -= dx*2;
        }
    }
    return num;
}

Eigen::Vector3f barycentric(const Point* vs, const Eigen::Vector2i& p) 
{ 
    // float c1 = (p.x()*(vs[1].y() - vs[2].y()) + (vs[2].x() - vs[1].x())*p.y() + vs[1].x()*vs[2].y() - vs[2].x()*vs[1].y()) / (vs[0].x()*(vs[1].y() - vs[2].y()) + (vs[2].x() - vs[1].x())*vs[0].y() + vs[1].x()*vs[2].y() - vs[2].x()*vs[1].y());
    // float c2 = (p.x()*(vs[2].y() - vs[0].y()) + (vs[0].x() - vs[2].x())*p.y() + vs[2].x()*vs[0].y() - vs[0].x()*vs[2].y()) / (vs[1].x()*(vs[2].y() - vs[0].y()) + (vs[0].x() - vs[2].x())*vs[1].y() + vs[2].x()*vs[0].y() - vs[0].x()*vs[2].y());
    // float c3 = (p.x()*(vs[0].y() - vs[1].y()) + (vs[1].x() - vs[0].x())*p.y() + vs[0].x()*vs[1].y() - vs[1].x()*vs[0].y()) / (vs[2].x()*(vs[0].y() - vs[1].y()) + (vs[1].x() - vs[0].x())*vs[2].y() + vs[0].x()*vs[1].y() - vs[1].x()*vs[0].y());
    // return Eigen::Vector3f(c1, c2, c3);
    Eigen::Vector3i u = Eigen::Vector3i(vs[2].screen_pos.x()-vs[0].screen_pos.x(), vs[1].screen_pos.x()-vs[0].screen_pos.x(), vs[0].screen_pos.x()-p.x()).cross(Eigen::Vector3i(vs[2].screen_pos.y()-vs[0].screen_pos.y(), vs[1].screen_pos.y()-vs[0].screen_pos.y(), vs[0].screen_pos.y()-p.y()));
    if (std::abs(u.z())<1) return Eigen::Vector3f(-1.f,1.f,1.f);
    return Eigen::Vector3f(1.f-(u.x()+u.y())/float(u.z()), u.y()/float(u.z()), u.x()/float(u.z())); 
}

bool Pipeline::test_depth(int x, int y, float z, int width, int height, float* z_buffer)
{
    int index = y * width + x;
    if(z < z_buffer[index])
    {
        z_buffer[index] = z;
        return true;
    }
    return false;
}

int triangle(const Point* input, Point* raster_region, int max_size, ShadeFrequency freq)
{
    Eigen::Vector2i box_min(4096, 2160); 
    Eigen::Vector2i box_max(0, 0);
    box_min.x() = std::min(input[0].screen_pos.x(), std::min(input[1].screen_pos.x(), input[2].screen_pos.x()));
    box_min.y() = std::min(input[0].screen_pos.y(), std::min(input[1].screen_pos.y(), input[2].screen_pos.y()));
    box_max.x() = std::max(input[0].screen_pos.x(), std::max(input[1].screen_pos.x(), input[2].screen_pos.x()));
    box_max.y() = std::max(input[0].screen_pos.y(), std::max(input[1].screen_pos.y(), input[2].screen_pos.y()));
    int num = 0;

    if(freq==ShadeFrequency::FLAT)
    {
        Eigen::Vector3f faceNormal = (input[2].v.pos-input[0].v.pos).cross(input[1].v.pos-input[0].v.pos).normalized();
        for(int x=box_min.x(); x<=box_max.x(); x++)
            for(int y=box_min.y(); y<=box_max.y(); y++)
            {
                Eigen::Vector3f bc_screen = barycentric(input, Eigen::Vector2i(x, y));
                if(bc_screen.x()<0 || bc_screen.y()<0 || bc_screen.z()<0)
                    continue;
                float z = bc_screen.x()*input[0].z + bc_screen.y()*input[1].z + bc_screen.z()*input[2].z;
                raster_region[num].screen_pos = {x, y};
                raster_region[num].z = z;
                raster_region[num].v.normal = faceNormal;
                num++;
                if(num > max_size) 
                {
                    return num;
                }
            }
    }

    return num;
}

}