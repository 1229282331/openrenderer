#pragma once

#include <iostream>
#include "Eigen/Eigen"
#include "geometry.hpp"

extern openrenderer::Uniform ubo;

namespace openrenderer{

inline Eigen::Vector3f point_VertexShader(const vertex_shader_in& input, vertex_shader_out& out_attr)
{
    Eigen::Vector4f pos = ubo.projection * ubo.view * ubo.models[input.obj_id] * Eigen::Vector4f(input.vertex.pos.x(), input.vertex.pos.y(), input.vertex.pos.z(), 1.f);
    pos /= pos[3];
    return Eigen::Vector3f(pos.x(), pos.y(), pos.z());
}

inline Eigen::Vector3f point_FragmentShader(const Point& input)
{
    return {1.f, 1.f, 1.f};
}

inline Eigen::Vector3f triangle_FragmentShader(const Point& input)
{
    Eigen::Vector3f color = {1.f, 1.f, 1.f};
    // float cos_a = ubo.faceNormal.dot(ubo.lightDir);
    float cos_a = input.v.normal.dot(ubo.lightDir);
    if(cos_a < 0.f)
        return {0.f, 0.f, 0.f};
    return cos_a * color;
}

inline Eigen::Vector3f depth_FragmentShader(const Point& input)
{
    float gray_value = input.z;
    return {gray_value, gray_value, gray_value};
}



}

