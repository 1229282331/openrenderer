#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include "Eigen/Eigen"
#include "Eigen/Dense"
#include "json/json.h"
#include "geometry.hpp"
#include "shader.hpp"

namespace openrenderer{

extern std::unordered_map<std::string, std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)>> vertexShaders_map;

enum class ConfigState{ OK=0, INCOMPLETE, ERROR };

class Config{
public:
    Config(const char* file);
    ~Config();

    ConfigState                                   state;
    std::unordered_map<std::string, unsigned int> obj_names;
    std::vector<std::string>                      obj_paths;
    std::vector<bool>                             enableColors;
    std::vector<Eigen::Vector3f>                  obj_colors;
    std::vector<Texture*>                         obj_colorTextures;
    std::vector<Texture*>                         obj_normalTextures;
    std::vector<Eigen::Matrix4f>                  obj_modelMatrixs;

    std::vector<Light>                            lights;
    int                                           width;
    int                                           height;
    Eigen::Vector3f                               cameraPos;
    Eigen::Vector3f                               lookAt;
    Eigen::Vector3f                               upDir;
    float                                         fovy;
    float                                         zNear;
    float                                         zFar;
    bool                                          enableDefferedRender;

    std::vector<std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)>> obj_vertexShaders;
    std::vector<std::function<Eigen::Vector3f(const Point&)>>                                obj_fragmentShaders;
};

}