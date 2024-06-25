#include "loader.hpp"
#define TINYOBJLOADER_IMPLEMENTATION    // define this for only *once*
#include "tiny_obj_loader.h"    

namespace openrenderer{

struct ObjHashFunc
{
    template<typename T, typename U>
    size_t operator() (const pair<T, U> &i) const
    {
        return hash<T>()(i.first) ^ hash<U>()(i.second);
    }
};

bool Loader::load_obj(const std::vector<std::string>& obj_paths, 
                std::vector<std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)>> vertexShaders,
                std::vector<std::function<Eigen::Vector3f(const Point&)>> fragmentShaders,
                std::vector<Texture*> pcolorTextures, std::vector<Texture*> pnormalTextures,
                const std::vector<Eigen::Matrix4f>& modelMats)
{
    for(int i=0; i<obj_paths.size(); i++)
    {
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string err;
        bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, obj_paths[i].data());
        printf("%s\n", err.c_str());
        if(!ret) 
        {
            printf("%s\n", err.c_str());
            return false;
        }

        Object obj;
        obj.id = i;
        std::unordered_map<std::pair<int, int>,int, ObjHashFunc> vertex_map;   // used to remove repeated vertex
        for(auto shape : shapes)    // loop all object meshes
        {
            if(shape.mesh.indices.size() > 1000)
            {
                for(auto idx : shape.mesh.indices)  // loop all indices
                {
                    Vertex vertex{};
                    vertex.pos = {
                        attrib.vertices[3*idx.vertex_index+0],
                        attrib.vertices[3*idx.vertex_index+1],
                        attrib.vertices[3*idx.vertex_index+2],
                    };
                    if(attrib.texcoords.size()!=0)
                    {
                        vertex.texCoord = {
                            attrib.texcoords[2*idx.texcoord_index+0],
                            attrib.texcoords[2*idx.texcoord_index+1],
                        };
                    }
                    else
                        vertex.texCoord = {0.f, 0.f};
                    if(attrib.normals.size()!=0)
                    {
                        vertex.normal = {
                            attrib.normals[3*idx.normal_index+0],
                            attrib.normals[3*idx.normal_index+1],
                            attrib.normals[3*idx.normal_index+2],
                        };
                    }
                    else
                        vertex.normal = {0.f, 0.f, 0.f};
                    vertex.color = {1.f, 1.f, 1.f};

                    std::pair<int, int>vandt_pair = {idx.vertex_index, idx.texcoord_index};
                    if(vertex_map.count(vandt_pair)==0)
                    {
                        vertex_map[vandt_pair] = static_cast<uint32_t>(obj.vertices.size());
                        obj.vertices.push_back(vertex);
                    }
                    obj.indices.push_back(vertex_map[vandt_pair]);
                }
            }
            else
            {
                for(auto idx : shape.mesh.indices)  // loop all indices
                {
                    Vertex vertex{};
                    vertex.pos = {
                        attrib.vertices[3*idx.vertex_index+0],
                        attrib.vertices[3*idx.vertex_index+1],
                        attrib.vertices[3*idx.vertex_index+2],
                    };
                    if(attrib.texcoords.size()!=0)
                    {
                        vertex.texCoord = {
                            attrib.texcoords[2*idx.texcoord_index+0],
                            attrib.texcoords[2*idx.texcoord_index+1],
                        };
                    }
                    else
                        vertex.texCoord = {0.f, 0.f};
                    if(attrib.normals.size()!=0)
                    {
                        vertex.normal = {
                            attrib.normals[3*idx.normal_index+0],
                            attrib.normals[3*idx.normal_index+1],
                            attrib.normals[3*idx.normal_index+2],
                        };
                    }
                    else
                        vertex.normal = {0.f, 0.f, 0.f};
                    vertex.color = {1.f, 1.f, 1.f};

                    std::pair<int, int>vandt_pair = {idx.vertex_index, idx.texcoord_index};
                    vertex_map[vandt_pair] = static_cast<uint32_t>(obj.vertices.size());
                    obj.vertices.push_back(vertex);
                    obj.indices.push_back(vertex_map[vandt_pair]);
                }
            }
        }
        obj.bounding_box = get_boundingBox(obj.vertices, true);
        objects.push_back(obj);
        printf("load object%d : vertices[#%d], faces[#%d]\n", obj.id, int(obj.vertices.size()), int(obj.indices.size())/3);
    }
    for(int i=0; i<modelMats.size() && i<obj_paths.size(); i++)
    {
        Eigen::Vector3f translateVec = Eigen::Vector3f(modelMats[i](0, 3), modelMats[i](1, 3), modelMats[i](2, 3));
        objects[i].bounding_box.p_min += translateVec;
        objects[i].bounding_box.p_max += translateVec;
        objects[i].modelMat = modelMats[i];
    }
    for(int i=0; i<vertexShaders.size() && i<obj_paths.size(); i++)
        objects[i].vertexShader = vertexShaders[i];
    for(int i=0; i<fragmentShaders.size() && i<obj_paths.size(); i++)
        objects[i].fragmentShader = fragmentShaders[i];
    for(int i=0; i<pcolorTextures.size() && i<obj_paths.size(); i++)
        objects[i].colorTexture = pcolorTextures[i];
    for(int i=0; i<pnormalTextures.size() && i<obj_paths.size(); i++)
        objects[i].normalTexture = pnormalTextures[i];

    return true;
}


Box get_boundingBox(const std::vector<Vertex>& vertexs, bool is_relax)
{
    Box box;
    box.p_min.x() = std::numeric_limits<float>::max();
    box.p_max.x() = std::numeric_limits<float>::min();
    box.p_min.y() = std::numeric_limits<float>::max();
    box.p_max.y() = std::numeric_limits<float>::min();
    box.p_min.z() = std::numeric_limits<float>::max();
    box.p_max.z() = std::numeric_limits<float>::min();

    for(int i=0; i<vertexs.size(); i++)
    {
        if(vertexs[i].pos.x() < box.p_min.x())  box.p_min.x() = vertexs[i].pos.x();
        if(vertexs[i].pos.x() > box.p_max.x())  box.p_max.x() = vertexs[i].pos.x();
        if(vertexs[i].pos.y() < box.p_min.y())  box.p_min.y() = vertexs[i].pos.y();
        if(vertexs[i].pos.y() > box.p_max.y())  box.p_max.y() = vertexs[i].pos.y();
        if(vertexs[i].pos.z() < box.p_min.z())  box.p_min.z() = vertexs[i].pos.z();
        if(vertexs[i].pos.z() > box.p_max.z())  box.p_max.z() = vertexs[i].pos.z();
    }

    if(is_relax)
    {
        float min = std::min(box.p_min.x(), std::min(box.p_min.y(), box.p_min.z()));
        float max = std::max(box.p_max.x(), std::max(box.p_max.y(), box.p_max.z()));
        box.p_min = {min, min, min};
        box.p_max = {max, max, max};
    }
    return box;
}



}

