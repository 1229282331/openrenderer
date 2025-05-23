#include "texture.hpp"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

namespace openrenderer{

Texture::Texture(const std::string& name)
{
    data = stbi_load(name.c_str(), &width, &height, &n_channels, 3);    //[RGBRGB...RGB]
    if(!data)
        printf("[error]: can't load the texture:%s\n", name.c_str());
}

Texture::~Texture() 
{ 
    stbi_image_free(data); 
}

Eigen::Vector3f Texture::getColor(float u, float v) //RGB
{
    int u_img = std::clamp(int(width * u), 0, width-1);
    int v_img = std::clamp(int(height * (1.f-v)), 0, height-1);
    // int u_img = int(width * u);
    // int v_img = int(height * (1.f-v));
    int pixelOffset = (width*v_img + u_img) * 3;
    return { data[pixelOffset]/255.f, data[pixelOffset+1]/255.f, data[pixelOffset+2]/255.f };
}


}
