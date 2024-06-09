#pragma once

#include <iostream>
#include "Eigen/Eigen"
#include "geometry.hpp"

extern openrenderer::Uniform ubo;

namespace openrenderer{



inline Eigen::Vector4f point_VertexShader(const vertex_shader_in& input, vertex_shader_out& out_attr)
{
    Eigen::Vector4f pos = ubo.projection * ubo.view * ubo.models[input.obj_id] * Eigen::Vector4f(input.vertex.pos.x(), input.vertex.pos.y(), input.vertex.pos.z(), 1.f);
    Eigen::Vector4f normal = ubo.models[input.obj_id] * Eigen::Vector4f(input.vertex.normal.x(), input.vertex.normal.y(), input.vertex.normal.z(), 0.f);
    out_attr.normal = Eigen::Vector3f(normal.x(), normal.y(), normal.z()).normalized();
    out_attr.position = Eigen::Vector3f{pos.x(), pos.y(), pos.z()} / pos.w();
    return pos;
}

inline Eigen::Vector4f nmap_VertexShader(const vertex_shader_in& input, vertex_shader_out& out_attr) 
{
    Eigen::Vector4f pos = ubo.projection * ubo.view * ubo.models[input.obj_id] * Eigen::Vector4f(input.vertex.pos.x(), input.vertex.pos.y(), input.vertex.pos.z(), 1.f);
    Eigen::Vector4f normal = ubo.models[input.obj_id] * Eigen::Vector4f(input.vertex.normal.x(), input.vertex.normal.y(), input.vertex.normal.z(), 0.f);
    out_attr.normal = Eigen::Vector3f(normal.x(), normal.y(), normal.z()).normalized();
    out_attr.position = Eigen::Vector3f{pos.x(), pos.y(), pos.z()} / pos.w();

    Eigen::Vector3f T = input.tangent;
    Eigen::Vector3f N = input.vertex.normal;
    Eigen::Vector4f T_vec = (ubo.models[input.obj_id] * Eigen::Vector4f(T.x(), T.y(), T.z(), 0.f)).normalized();
    Eigen::Vector4f N_vec = (ubo.models[input.obj_id] * Eigen::Vector4f(N.x(), N.y(), N.z(), 0.f)).normalized();
    T.x() = T_vec.x(), T.y() = T_vec.y(), T.z() = T_vec.z();
    N.x() = N_vec.x(), N.y() = N_vec.y(), N.z() = N_vec.z();
    T = (T - T.dot(N) * N).normalized();
    Eigen::Vector3f B = N.cross(T).normalized();
    out_attr.TBN.col(0) = T;
    out_attr.TBN.col(1) = B;
    out_attr.TBN.col(2) = N;
    return pos;
}

inline float getShadowBias(float c, int light_id, const Point& input)
{
    Eigen::Vector3f lightDir = (ubo.lights[light_id].pos - input.v.pos).normalized();
    return lightDir.dot(input.attrs.normal) * c;
}
inline float isVisible(const Buffer* shadowMap, Eigen::Vector4f shadowCoord, float bias, int width, int height, const Point& input)
{
    Float32ToUint8 value;
    int x = int((shadowCoord.x()/shadowCoord.w()+1.f)*float(width)/2.f);
    int y = int((shadowCoord.y()/shadowCoord.w()+1.f)*float(height)/2.f);
    value.arr[0] = shadowMap->get(y, x, ColorBit::B);
    value.arr[1] = shadowMap->get(y, x, ColorBit::G);
    value.arr[2] = shadowMap->get(y, x, ColorBit::R);
    value.arr[3] = shadowMap->get(y, x, ColorBit::A);
    float lightPass_depth = value.num;
    float cameraPass_depth = shadowCoord.z() / shadowCoord.w();
    if(lightPass_depth < cameraPass_depth - bias)
        return 0.f;
    return 1.f;
}
inline float calVisibility(const std::vector<Light>& lights, const Point& input)
{
    // get shading point position from light, calculate the visibility for this point
    float visibility = 0.f;
    int shadows_num = 0;
    for(int i=0; i<lights.size(); i++)
    {
        if(lights[i].hasShadowMap)
        {
            Eigen::Vector4f posFromLight = lights[i].lightVP * ubo.models[input.obj_id] * Eigen::Vector4f{input.v.pos.x(), input.v.pos.y(), input.v.pos.z(), 1.f};
            float bias = getShadowBias(0.001f, i, input);
            visibility += isVisible(lights[i].shadowMap, posFromLight, bias, ubo.width, ubo.height, input);
            shadows_num++;
        }
    }
    if(shadows_num==0)
        visibility = 1.f;
    else
        visibility /= float(shadows_num);
    return visibility;
}

inline Eigen::Vector3f point_FragmentShader(const Point& input)
{
    float visibility = calVisibility(ubo.lights, input);
    return visibility * Eigen::Vector3f{ 0.5f, 0.5f, 0.5f };
}

inline Eigen::Vector3f triangle_FragmentShader(const Point& input)
{
    float visibility = calVisibility(ubo.lights, input);
    Eigen::Vector3f color = {1.f, 1.f, 1.f};
    // float cos_a = ubo.faceNormal.dot(ubo.lightDir);
    float cos_a = input.v.normal.normalized().dot(-ubo.lightDir);
    if(cos_a < 0.f)
        return {0.f, 0.f, 0.f};
    return visibility * cos_a * color;
}

inline Eigen::Vector3f normal_FragmentShader(const Point& input)
{
    Eigen::Vector3f color = (input.v.normal.normalized() + Eigen::Vector3f(1.f, 1.f, 1.f))/2.f;
    return color;
}

inline Eigen::Vector3f depth_FragmentShader(const Point& input)
{
    float gray_value =  input.attrs.position.z();   // no-linear
    // float gray_value =  input.z / 1000.f;     // linear
    // std::cout << gray_value << '\n';

    return {gray_value, gray_value, gray_value};
}

inline Eigen::Vector3f texture_FragmentShader(const Point& input)
{
    Eigen::Vector3f color = input.colorTexture->getColor(input.v.texCoord.x(), input.v.texCoord.y());

    return color;
}

inline Eigen::Vector3f phong_FragmentShader(const Point& input)
{
    float visibility = calVisibility(ubo.lights, input);
    Eigen::Vector3f ka = {0.005f, 0.005f,0.005f};
    Eigen::Vector3f kd = input.colorTexture->getColor(input.v.texCoord.x(), input.v.texCoord.y());
    Eigen::Vector3f ks = {0.7937f, 0.7937f, 0.7937f};
    Eigen::Vector3f amb_intensity = {20.f, 20.f, 20.f};
    float p = 250.f;

    Eigen::Vector3f La = ka.cwiseProduct(amb_intensity);
    Eigen::Vector3f color = La;
    Eigen::Vector3f n = input.attrs.normal.normalized();
    for(int i=0; i<ubo.lights.size(); i++)
    {
        Eigen::Vector3f l = (ubo.lights[i].pos-input.v.pos).normalized();
        Eigen::Vector3f v = (ubo.cameraPos-input.v.pos).normalized();
        Eigen::Vector3f h = (l+v).normalized();
        float r2_inverse = 1.f / std::pow((ubo.lights[i].pos-input.v.pos).norm(), 2.f);
        float cos_a = std::max(0.f, n.dot(h));

        Eigen::Vector3f Ld = kd.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * cos_a;
        Eigen::Vector3f Ls = ks.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * pow(cos_a, p);
        color += (Ld + Ls);
    }

    return visibility * color;
}

inline Eigen::Vector3f normalMapping_FragmentShader(const Point& input)
{
    float visibility = calVisibility(ubo.lights, input);
    Eigen::Vector3f ka = {0.005f, 0.005f,0.005f};
    Eigen::Vector3f kd = input.colorTexture->getColor(input.v.texCoord.x(), input.v.texCoord.y());
    Eigen::Vector3f ks = {0.7937f, 0.7937f, 0.7937f};
    Eigen::Vector3f amb_intensity = {20.f, 20.f, 20.f};
    float p = 250.f;

    Eigen::Vector3f normal = (input.attrs.TBN * (input.normalTexture->getColor(input.v.texCoord.x(), input.v.texCoord.y()) * 2.f - Eigen::Vector3f{1.f, 1.f, 1.f}).normalized()).normalized();
    Eigen::Vector3f La = ka.cwiseProduct(amb_intensity);
    Eigen::Vector3f color = La;
    for(int i=0; i<ubo.lights.size(); i++)
    {
        Eigen::Vector3f l = (ubo.lights[i].pos-input.v.pos).normalized();
        Eigen::Vector3f v = (ubo.cameraPos-input.v.pos).normalized();
        Eigen::Vector3f h = (l+v).normalized();
        float r2_inverse = 1.f / std::pow((ubo.lights[i].pos-input.v.pos).norm(), 2.f);

        Eigen::Vector3f Ld = kd.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * std::max(0.f, normal.dot(h));
        Eigen::Vector3f Ls = ks.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * pow(std::max(0.f, normal.dot(h)), p);
        color += (Ld + Ls);
    }
    return visibility * color;
}

inline Eigen::Vector3f bumpMapping_FragmentShader(const Point& input)
{
    float visibility = calVisibility(ubo.lights, input);
    float kh = 0.7f, kn = 0.7f;
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    float u_ = input.v.texCoord.x();   //u->[0,1]
    float v_ = input.v.texCoord.y();   //v->[0,1]
    float dU = kh * (input.normalTexture->getColor(u_+(1.0f/ubo.width), v_).norm() - input.normalTexture->getColor(u_, v_).norm());
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    float dV = kn * (input.normalTexture->getColor(u_, v_+(1.0f/ubo.height)).norm() - input.normalTexture->getColor(u_, v_).norm());
    // Vector ln = (-dU, -dV, 1)
    Eigen::Vector3f ln = {-dU, -dV, 1.0f};
    // Normal n = normalize(TBN * ln)
    Eigen::Vector3f normal = (input.attrs.TBN * ln).normalized();


    Eigen::Vector3f ka = {0.005f, 0.005f,0.005f};
    Eigen::Vector3f kd = input.colorTexture->getColor(input.v.texCoord.x(), input.v.texCoord.y());
    Eigen::Vector3f ks = {0.7937f, 0.7937f, 0.7937f};
    Eigen::Vector3f amb_intensity = {20.f, 20.f, 20.f};
    float p = 250.f;

    Eigen::Vector3f La = ka.cwiseProduct(amb_intensity);
    Eigen::Vector3f color = La;
    for(int i=0; i<ubo.lights.size(); i++)
    {
        Eigen::Vector3f l = (ubo.lights[i].pos-input.v.pos).normalized();
        Eigen::Vector3f v = (ubo.cameraPos-input.v.pos).normalized();
        Eigen::Vector3f h = (l+v).normalized();
        float r2_inverse = 1.f / std::pow((ubo.lights[i].pos-input.v.pos).norm(), 2.f);

        Eigen::Vector3f Ld = kd.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * std::max(0.f, normal.dot(h));
        Eigen::Vector3f Ls = ks.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * pow(std::max(0.f, normal.dot(h)), p);
        color += (Ld + Ls);
    }
    return visibility * color;
}

inline Eigen::Vector3f displaceMapping_FragmentShader(const Point& input)
{
    float visibility = calVisibility(ubo.lights, input);
    float kh = 0.3f, kn = 0.3f;
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    float u_ = input.v.texCoord.x();   //u->[0,1]
    float v_ = input.v.texCoord.y();   //v->[0,1]
    float dU = kh * (input.normalTexture->getColor(u_+(1.0f/ubo.width), v_).norm() - input.normalTexture->getColor(u_, v_).norm());
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    float dV = kn * (input.normalTexture->getColor(u_, v_+(1.0f/ubo.height)).norm() - input.normalTexture->getColor(u_, v_).norm());
    // Vector ln = (-dU, -dV, 1)
    Eigen::Vector3f ln = {-dU, -dV, 1.0f};
    // Normal n = normalize(TBN * ln)
    Eigen::Vector3f normal = (input.attrs.TBN * ln).normalized();
    Eigen::Vector3f position = input.attrs.position + kn * normal * input.normalTexture->getColor(u_, v_).norm();

    Eigen::Vector3f ka = {0.005f, 0.005f,0.005f};
    Eigen::Vector3f kd = input.colorTexture->getColor(input.v.texCoord.x(), input.v.texCoord.y());
    Eigen::Vector3f ks = {0.7937f, 0.7937f, 0.7937f};
    Eigen::Vector3f amb_intensity = {20.f, 20.f, 20.f};
    float p = 250.f;

    Eigen::Vector3f La = ka.cwiseProduct(amb_intensity);
    Eigen::Vector3f color = La;
    for(int i=0; i<ubo.lights.size(); i++)
    {
        Eigen::Vector3f l = (ubo.lights[i].pos-position).normalized();
        Eigen::Vector3f v = (ubo.cameraPos-position).normalized();
        Eigen::Vector3f h = (l+v).normalized();
        float r2_inverse = 1.f / std::pow((ubo.lights[i].pos-position).norm(), 2.f);

        Eigen::Vector3f Ld = kd.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * std::max(0.f, normal.dot(h));
        Eigen::Vector3f Ls = ks.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * pow(std::max(0.f, normal.dot(h)), p);
        color += (Ld + Ls);
    }

    return visibility * color;
}

}

