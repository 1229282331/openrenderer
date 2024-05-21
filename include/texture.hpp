#pragma once

#include <string>
#include "Eigen/Eigen"
#include "Eigen/Dense"


namespace openrenderer{

class Texture{
public:
    Texture(const std::string& name);
    ~Texture();

    Eigen::Vector3f getColor(float u, float v);

private:
    uint8_t* data;
    int width;
    int height;
    int n_channels;
};


}