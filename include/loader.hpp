#pragma once

#include <array>
#include <vector>
#include <string>
#include <array>
#include <unordered_map>
#include "geometry.hpp"
#include "Eigen/Core"
#include "shader.hpp"
#include "texture.hpp"
#include "utils.hpp"


namespace openrenderer{

struct Box{
    Eigen::Vector3f p_min;
    Eigen::Vector3f p_max;
    float volume() { return (p_max[0]-p_min[0])*(p_max[1]-p_min[1])*(p_max[2]-p_min[2]); }

    bool intersectP(const Eigen::Vector3f& origin, const Eigen::Vector3f& dir)
    {
        float tx_inter = (p_min.x()-origin.x()) / dir.x();
        float tx_exit = (p_max.x()-origin.x()) / dir.x();
        if(dir.x()<0.f) std::swap(tx_inter, tx_exit);
        float ty_inter = (p_min.y()-origin.y()) / dir.y();
        float ty_exit = (p_max.y()-origin.y()) / dir.y();
        if(dir.y()<0.f) std::swap(ty_inter, ty_exit);
        float tz_inter = (p_min.z()-origin.z()) / dir.z();
        float tz_exit = (p_max.z()-origin.z()) / dir.z();
        if(dir.z()<0.f) std::swap(tz_inter, tz_exit);

        float t_enter = std::max(tx_inter, std::max(ty_inter, tz_inter));
        float t_exit = std::min(tx_exit, std::min(ty_exit, tz_exit));
        return t_enter<t_exit && t_exit>=0;
    }
};

struct Object{
    int id;
    Light* light = nullptr;
    Box bounding_box;
    std::vector<Vertex> vertices;    
    std::vector<int> indices;
    Eigen::Matrix4f modelMat = Eigen::Matrix4f::Identity();
    std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)> vertexShader = point_VertexShader;
    std::function<Eigen::Vector3f(const Point&)> fragmentShader = point_FragmentShader;
    Texture* colorTexture = nullptr;
    Texture* normalTexture = nullptr;
};

class Loader{
public:
    Loader() = default;
    ~Loader() = default;
    bool load_obj(const std::vector<std::string>& obj_paths, 
                    std::vector<std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)>> vertexShaders,
                    std::vector<std::function<Eigen::Vector3f(const Point&)>> fragmentShaders,
                    std::vector<Texture*> pcolorTextures, std::vector<Eigen::Vector3f> obj_colors, std::vector<Texture*> pnormalTextures,
                    const std::vector<Eigen::Matrix4f>& modelMats=std::vector<Eigen::Matrix4f>());

    std::vector<Object> objects;
    std::vector<std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)>> vertexShaders;
    std::vector<std::function<Eigen::Vector3f(const Point&)>> fragmentShaders;

private:

};

Box get_boundingBox(const std::vector<Vertex>& vertexs, bool is_relax=false);

}




