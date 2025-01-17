#pragma once

#include <string>
#include "Eigen/Eigen"
#include "Eigen/Dense"
#include "buffer.hpp"


namespace openrenderer{

class Texture{
public:
    Texture(const std::string& name);
    ~Texture();

    Eigen::Vector3f getColor(float u, float v);

public:
    uint8_t* data;
    int width;
    int height;
    int n_channels;
};


}