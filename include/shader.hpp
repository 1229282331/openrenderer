#pragma once

#include <iostream>
#include "Eigen/Eigen"
#include "geometry.hpp"

extern openrenderer::Uniform ubo;
#define EPS 6e-4
#define NUM_SAMPLES 16

namespace openrenderer{

inline Eigen::Vector4f point_VertexShader(const vertex_shader_in& input, vertex_shader_out& out_attr)
{
    Eigen::Vector4f modelPos = ubo.models[input.obj_id] * Eigen::Vector4f(input.vertex.pos.x(), input.vertex.pos.y(), input.vertex.pos.z(), 1.f);
    Eigen::Vector4f pos = ubo.projection * ubo.view * modelPos;
    Eigen::Vector4f normal = ubo.models[input.obj_id] * Eigen::Vector4f(input.vertex.normal.x(), input.vertex.normal.y(), input.vertex.normal.z(), 0.f);
    out_attr.normal = Eigen::Vector3f(normal.x(), normal.y(), normal.z()).normalized();
    out_attr.ndcPos = Eigen::Vector3f{pos.x(), pos.y(), pos.z()} / pos.w();
    out_attr.modelPos = Eigen::Vector3f{modelPos.x(), modelPos.y(), modelPos.z()};
    return pos;
}

inline Eigen::Vector4f nmap_VertexShader(const vertex_shader_in& input, vertex_shader_out& out_attr) 
{
    Eigen::Vector4f modelPos = ubo.models[input.obj_id] * Eigen::Vector4f(input.vertex.pos.x(), input.vertex.pos.y(), input.vertex.pos.z(), 1.f);
    Eigen::Vector4f pos = ubo.projection * ubo.view * modelPos;
    Eigen::Vector4f normal = ubo.models[input.obj_id] * Eigen::Vector4f(input.vertex.normal.x(), input.vertex.normal.y(), input.vertex.normal.z(), 0.f);
    out_attr.normal = Eigen::Vector3f(normal.x(), normal.y(), normal.z()).normalized();
    out_attr.ndcPos = Eigen::Vector3f{pos.x(), pos.y(), pos.z()} / pos.w();
    out_attr.modelPos = Eigen::Vector3f{modelPos.x(), modelPos.y(), modelPos.z()};

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

inline Eigen::Vector2i worldPos2screenPos(Eigen::Vector3f worldPos)
{
    Eigen::Vector4f ndcPos = ubo.projection * ubo.view * Eigen::Vector4f{worldPos.x(), worldPos.y(), worldPos.z(), 1.f};
    int x = int((ndcPos.x()/ndcPos.w()+1.f)*float(ubo.width)/2.f);
    int y = int((ndcPos.y()/ndcPos.w()+1.f)*float(ubo.height)/2.f);
    return Eigen::Vector2i{x, y};
}
inline float depth2Linear(float z, float zNear, float zFar)
{
    return zNear + (zFar-zNear)*z;
}

inline float rand1(float& p) 
{
    float tmp;
    p = modff(p * 0.1031f, &tmp);
    p *= p + 33.33f;
    p *= p + p;
    return modff(p, &tmp);
}
inline Eigen::Vector2f rand2(float& p)
{
    float x1 = rand1(p);
    float x2 = rand1(p);
    return { x1, x2 };
}
inline float initRand(Eigen::Vector2f uv) 
{
    float tmp;
	Eigen::Vector3f p3  = {modff(uv.x()*0.1031f, &tmp), modff(uv.y()*0.1031f, &tmp), modff(uv.x()*0.1031f, &tmp)};
    float x = p3.dot(Eigen::Vector3f{p3.y(), p3.z(), p3.x()}+Eigen::Vector3f{33.33f, 33.33f, 33.33f});
    p3 += Eigen::Vector3f{x, x, x};
    return modff((p3.x()+p3.y())*p3.z(), &tmp);
}
inline float rand_1to1(float x) // return [-1, 1]
{
    float tmp;
    return modff(sin(x)*10000.f, &tmp);
}
inline float rand_2to1(const Eigen::Vector2f& uv)  // return [0, 1]
{ 
    float tmp;
	const float a = 12.9898f, b = 78.233f, c = 43758.5453f;
	float dt = uv.dot(Eigen::Vector2f{a, b}), sn = fmod( dt, float(MY_PI) );
	return modff(sin(sn) * c, &tmp);
}
inline void uniformDiskSamples(const Eigen::Vector2f& randomSeed, int num_samples, std::vector<Eigen::Vector2f>& sampleDisk ) 
{
    float randNum = rand_2to1(randomSeed);
    float sampleX = rand_1to1( randNum ) ;
    float sampleY = rand_1to1( sampleX ) ;

    float angle = sampleX * float(MY_2PI);
    float radius = sqrt(abs(sampleY));

    for( int i = 0; i < num_samples; i ++ ) 
    {
        sampleDisk[i] = Eigen::Vector2f{radius * cos(angle) , radius * sin(angle)};

        sampleX = rand_1to1( sampleY );
        sampleY = rand_1to1( sampleX );

        angle = sampleX * float(MY_2PI);
        radius = sqrt(abs(sampleY));
    }
}
inline void poissonDiskSamples(const Eigen::Vector2f& randomSeed, int num_rings, int num_samples, std::vector<Eigen::Vector2f>& sampleDisk ) 
{
    float ANGLE_STEP = float(MY_2PI) * float( num_rings ) / float( num_samples );
    float INV_NUM_SAMPLES = 1.f / float( num_samples );

    float angle = rand_2to1( randomSeed ) * float(MY_2PI);
    float radius = INV_NUM_SAMPLES;
    float radiusStep = radius;

    for( int i = 0; i < num_samples; i ++ ) 
    {
        sampleDisk[i] = Eigen::Vector2f{cos(angle), sin(angle)} * pow(radius, 0.75);
        radius += radiusStep;
        angle += ANGLE_STEP;
    }
}

inline float findBlocker(const Buffer<uint8_t>* shadowMap, Eigen::Vector4f shadowCoord, float light_size, float bias, int width, int height, int num_samples, int shadowmap_resolution)
{
    // 1.determine the filter region size according the frustum
    float linear_z = depth2Linear(shadowCoord.z(), ubo.shadowmap_zNear, ubo.shadowmap_zFar);
    float filter_size = (linear_z-ubo.shadowmap_zNear) / linear_z * light_size;
    // 2.calculate the average depth for the blocker
    float prec = filter_size / float(shadowmap_resolution);
    Float32ToUint8 value;
    float average_blockerDepth = 0.f;
    int count = 0;
    std::vector<Eigen::Vector2f> sampleDisk(num_samples);
    poissonDiskSamples({shadowCoord.x(), shadowCoord.y()}, 10, num_samples, sampleDisk);
    for(int i=0; i<num_samples; i++)
    {
        float delta_x = 0.5f * sampleDisk[i].x() * prec;
        float delta_y = 0.5f * sampleDisk[i].y() * prec;
        int x = int((shadowCoord.x()+delta_x+1.f)*float(width)/2.f);
        int y = int((shadowCoord.y()+delta_y+1.f)*float(height)/2.f);
        value.arr[0] = shadowMap->get(y, x, ColorBit::B);
        value.arr[1] = shadowMap->get(y, x, ColorBit::G);
        value.arr[2] = shadowMap->get(y, x, ColorBit::R);
        value.arr[3] = shadowMap->get(y, x, ColorBit::A);
        float sample_depth = value.num;
        if(sample_depth < shadowCoord.z() - bias - EPS)
        {
            average_blockerDepth += sample_depth;
            count++;
        }
    }
    if(count==0)
        return 1.f;
    average_blockerDepth /= float(count);
    return average_blockerDepth;
}
inline Eigen::Vector3f sampleHemisphereUniform(float& seed, float& pdf) 
{
    Eigen::Vector2f uv = rand2(seed);
    float z = uv.x();
    float phi = uv.y() * float(MY_2PI);
    float sinTheta = sqrt(1.0f - z*z);
    Eigen::Vector3f dir = Eigen::Vector3f{sinTheta * cos(phi), sinTheta * sin(phi), z};
    pdf = float(INV_2PI);
    return dir;
}
inline float getShadowBias(float c, int light_id, const Point& input)
{
    Eigen::Vector3f lightDir = (ubo.lights[light_id].pos - input.attrs.modelPos).normalized();
    return lightDir.dot(input.attrs.normal) * c;
}
inline float isVisible(const Buffer<uint8_t>* shadowMap, Eigen::Vector4f shadowCoord, float bias, int width, int height, const Point& input)
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
    if(lightPass_depth>0.f && lightPass_depth < cameraPass_depth - bias - EPS)  // front of the light
        return 0.f;
    return 1.f;
}
inline float PCF(const Buffer<uint8_t>* shadowMap, Eigen::Vector4f shadowCoord, float bias, int width, int height, const Point& input, 
                    const char* sample_method="nearest", int shadowmap_resolution=720, int num_samples=16, float filter_size=5.f)
{
    int count = 0;
    float prec = filter_size / float(shadowmap_resolution);
    int weight = num_samples / 16;   // the weight of the shading point
    if(weight < 1)  weight = 1;
    shadowCoord /= shadowCoord.w();

    // 1. calculate the shading point visibility
    Float32ToUint8 value;
    int x = int((shadowCoord.x()+1.f)*float(width)/2.f);
    int y = int((shadowCoord.y()+1.f)*float(height)/2.f);
    value.arr[0] = shadowMap->get(y, x, ColorBit::B);
    value.arr[1] = shadowMap->get(y, x, ColorBit::G);
    value.arr[2] = shadowMap->get(y, x, ColorBit::R);
    value.arr[3] = shadowMap->get(y, x, ColorBit::A);
    float lightPass_depth = value.num;
    float cameraPass_depth = shadowCoord.z();
    if(lightPass_depth <= 0.f)  // back of the light
        return 1.f;
    if(lightPass_depth >= cameraPass_depth - bias - EPS)
        count += weight;

    // 2. sample points around the shading point
    if(strcmp(sample_method, "nearest")==0)
    {
        int sample_edge = int(filter_size/2) + 1;
        std::vector<float> delta(sample_edge);
        for(int k=0; k<sample_edge; k++)
            delta[k] = -prec/2.f + prec/float(sample_edge)*float(k); 
        for(int i=0; i<sample_edge; i++)
            for(int j=0; j<sample_edge; j++)
            {
                int x = int((shadowCoord.x()+delta[i]+1.f)*float(width)/2.f);
                int y = int((shadowCoord.y()+delta[j]+1.f)*float(height)/2.f);
                value.arr[0] = shadowMap->get(y, x, ColorBit::B);
                value.arr[1] = shadowMap->get(y, x, ColorBit::G);
                value.arr[2] = shadowMap->get(y, x, ColorBit::R);
                value.arr[3] = shadowMap->get(y, x, ColorBit::A);
                float sample_depth = value.num;
                if(sample_depth >= cameraPass_depth - bias - EPS)
                    count++;
            }
        return count / float(sample_edge*sample_edge+weight);
    }
    else if(strcmp(sample_method, "random")==0)
    {
        for(int i=0; i<num_samples; i++)
        {
            float delta_x = 0.5f * rand_1to1(shadowCoord.x()) * prec;
            float delta_y = 0.5f * rand_1to1(shadowCoord.y()) * prec;
            int x = int((shadowCoord.x()+delta_x+1.f)*float(width)/2.f);
            int y = int((shadowCoord.y()+delta_y+1.f)*float(height)/2.f);
            value.arr[0] = shadowMap->get(y, x, ColorBit::B);
            value.arr[1] = shadowMap->get(y, x, ColorBit::G);
            value.arr[2] = shadowMap->get(y, x, ColorBit::R);
            value.arr[3] = shadowMap->get(y, x, ColorBit::A);
            float sample_depth = value.num;
            if(sample_depth >= cameraPass_depth - bias - EPS)
                count++;
        }
        return count / float(num_samples+weight);
    }
    else if(strcmp(sample_method, "uniformDisk")==0)
    {
        std::vector<Eigen::Vector2f> sampleDisk(num_samples);
        uniformDiskSamples({shadowCoord.x(), shadowCoord.y()}, num_samples, sampleDisk);
        for(int i=0; i<num_samples; i++)
        {
            float delta_x = 0.5f * sampleDisk[i].x() * prec;
            float delta_y = 0.5f * sampleDisk[i].y() * prec;
            int x = int((shadowCoord.x()+delta_x+1.f)*float(width)/2.f);
            int y = int((shadowCoord.y()+delta_y+1.f)*float(height)/2.f);
            value.arr[0] = shadowMap->get(y, x, ColorBit::B);
            value.arr[1] = shadowMap->get(y, x, ColorBit::G);
            value.arr[2] = shadowMap->get(y, x, ColorBit::R);
            value.arr[3] = shadowMap->get(y, x, ColorBit::A);
            float sample_depth = value.num;
            if(sample_depth >= cameraPass_depth - bias - EPS)
                count++;
        }
        return count / float(num_samples+weight);
    }
    else if(strcmp(sample_method, "poissonDisk")==0)
    {
        std::vector<Eigen::Vector2f> sampleDisk(num_samples);
        poissonDiskSamples({shadowCoord.x(), shadowCoord.y()}, 10, num_samples, sampleDisk);
        for(int i=0; i<num_samples; i++)
        {
            float delta_x = 0.5f * sampleDisk[i].x() * prec;
            float delta_y = 0.5f * sampleDisk[i].y() * prec;
            int x = int((shadowCoord.x()+delta_x+1.f)*float(width)/2.f);
            int y = int((shadowCoord.y()+delta_y+1.f)*float(height)/2.f);
            value.arr[0] = shadowMap->get(y, x, ColorBit::B);
            value.arr[1] = shadowMap->get(y, x, ColorBit::G);
            value.arr[2] = shadowMap->get(y, x, ColorBit::R);
            value.arr[3] = shadowMap->get(y, x, ColorBit::A);
            float sample_depth = value.num;
            if(sample_depth >= cameraPass_depth - bias - EPS)
                count++;
        }
        return count / float(num_samples+weight);
    }
    else
    {
        int sample_edge = int(filter_size/2) + 1;
        std::vector<float> delta(sample_edge);
        for(int k=0; k<sample_edge; k++)
            delta[k] = -prec/2.f + prec/float(sample_edge)*float(k); 
        for(int i=0; i<sample_edge; i++)
            for(int j=0; j<sample_edge; j++)
            {
                int x = int((shadowCoord.x()+delta[i]+1.f)*float(width)/2.f);
                int y = int((shadowCoord.y()+delta[j]+1.f)*float(height)/2.f);
                value.arr[0] = shadowMap->get(y, x, ColorBit::B);
                value.arr[1] = shadowMap->get(y, x, ColorBit::G);
                value.arr[2] = shadowMap->get(y, x, ColorBit::R);
                value.arr[3] = shadowMap->get(y, x, ColorBit::A);
                float sample_depth = value.num;
                if(sample_depth >= cameraPass_depth - bias - EPS)
                    count++;
            }
        return count / float(sample_edge*sample_edge+weight);
    }
}
inline float PCSS(const Buffer<uint8_t>* shadowMap, Eigen::Vector4f shadowCoord, float bias, int width, int height, const Point& input, 
                    const char* sample_method="nearest", int shadowmap_resolution=720, int num_samples=16)
{
    float light_size = 10.f;
    // STEP 1: avgblocker depth
    float blocker_depth = findBlocker(shadowMap, shadowCoord, light_size, bias, width, height, num_samples, shadowmap_resolution);
    if(blocker_depth > 1.0-EPS || blocker_depth<=0.f) return 1.0f;  // back of the light
    else if(blocker_depth < EPS) return 0.f;

    // STEP 2: penumbra size
    float penumbra_size = depth2Linear((shadowCoord.z()-blocker_depth)/blocker_depth, ubo.shadowmap_zNear, ubo.shadowmap_zFar) * light_size;

    // STEP 3: filtering
    return PCF(shadowMap, shadowCoord, bias, width, height, input, sample_method, shadowmap_resolution, num_samples, penumbra_size);
}
inline float calVisibility(const std::vector<Light>& lights, const Point& input, const char* method="hard")
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
            if(strcmp(method, "hard")==0)
                visibility += isVisible(lights[i].shadowMap, posFromLight, bias, ubo.width, ubo.height, input);
            else if(strcmp(method, "pcf")==0)
                visibility += PCF(lights[i].shadowMap, posFromLight, bias, ubo.width, ubo.height, input, "poissonDisk", 1000, NUM_SAMPLES, 10.f);
            else if(strcmp(method, "pcss")==0)
                visibility += PCSS(lights[i].shadowMap, posFromLight, bias, ubo.width, ubo.height, input, "poissonDisk", 1000, NUM_SAMPLES);
            else
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
inline float calSSAO(const Point& input, float radius=0.05f)
{
    if(!input.gbuffers)
        return 0.f;
    float occlusion = 0.f;
    float sign = ubo.cameraPos.z()<0.f ? 1.f : -1.f;
    Eigen::Vector3f worldPos = Eigen::Vector3f{ (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::B),
                                               (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::G),
                                               (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::R) };
    Eigen::Vector3f normal = Eigen::Vector3f{ (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::B),
                                               (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::G),
                                               (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::R) };
    for(int i=0; i<ubo.sampleFromHalfSphere.size(); i++)
    {
        Eigen::Vector3f samplePos = input.attrs.TBN * ubo.sampleFromHalfSphere[i].normalized();
        samplePos = worldPos + samplePos.normalized() * radius;
        Eigen::Vector4f offset{samplePos.x(), samplePos.y(), samplePos.z(), 1.f};
        Eigen::Vector4f viewPos = ubo.view * offset;
        Eigen::Vector4f ndcPos = ubo.projection * viewPos;
        int x = int((ndcPos.x()/ndcPos.w()+1.f)*float(ubo.width)/2.f);
        int y = int((ndcPos.y()/ndcPos.w()+1.f)*float(ubo.height)/2.f);
        float depth = -(*input.gbuffers->position_buffer)(y, x, ColorBit::A);
        float sample_depth = -viewPos.z() / viewPos.w();
        // std::cout << sample_depth << ',' << depth << '\n';
        if(sample_depth > depth + 5e-2)
            occlusion += 1.f;
    }
    occlusion /= float(ubo.sampleFromHalfSphere.size());
    return occlusion;
}
inline Eigen::Vector3f calSSDO(const Point& input, float radius=0.05f)
{
    if(!input.gbuffers)
        return {0.1f, 0.1f, 0.1f};
    Eigen::Vector3f ambient = {0.f, 0.f, 0.f};
    float sign = ubo.cameraPos.z()<0.f ? 1.f : -1.f;
    Eigen::Vector3f worldPos = Eigen::Vector3f{ (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::B),
                                               (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::G),
                                               (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::R) };
    Eigen::Vector3f normal = Eigen::Vector3f{ (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::B),
                                               (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::G),
                                               (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::R) };
    for(int i=0; i<ubo.sampleFromHalfSphere.size(); i++)
    {
        Eigen::Vector3f samplePos = input.attrs.TBN * ubo.sampleFromHalfSphere[i].normalized();
        samplePos = worldPos + samplePos.normalized() * radius;
        Eigen::Vector4f offset{samplePos.x(), samplePos.y(), samplePos.z(), 1.f};
        Eigen::Vector4f viewPos = ubo.view * offset;
        Eigen::Vector4f ndcPos = ubo.projection * viewPos;
        int x = int((ndcPos.x()/ndcPos.w()+1.f)*float(ubo.width)/2.f);
        int y = int((ndcPos.y()/ndcPos.w()+1.f)*float(ubo.height)/2.f);
        float depth = -(*input.gbuffers->position_buffer)(y, x, ColorBit::A);
        float sample_depth = -viewPos.z() / viewPos.w();
        Eigen::Vector3f albedo = Eigen::Vector3f{ (*input.gbuffers->albedo_buffer)(y, x, ColorBit::B),
                                               (*input.gbuffers->albedo_buffer)(y, x, ColorBit::G),
                                               (*input.gbuffers->albedo_buffer)(y, x, ColorBit::R) };
        Eigen::Vector3f l = (samplePos-worldPos).normalized();
        Eigen::Vector3f v = (ubo.cameraPos-worldPos).normalized();
        Eigen::Vector3f h = (l+v).normalized();
        float r2_inverse = 1.f / std::pow((samplePos-worldPos).norm(), 2.f);
        float cos_a = std::max(0.f, normal.dot(h));
        albedo = albedo * cos_a;
        if(sample_depth > depth + 5e-2)
            ambient += albedo;
    }
    ambient /= float(ubo.sampleFromHalfSphere.size());
    return ambient;
}
inline bool rayMarch(Eigen::Vector3f ori, Eigen::Vector3f dir, Eigen::Vector3f& hitPos, const Point& input)
{
    float step_size = 0.01f;
    int max_search_time = 1000;

    Eigen::Vector3f current_ori = ori;
    Eigen::Vector3f step = step_size * dir;
    current_ori += step;
    for(int i=0; i<max_search_time; i++)
    {
        Eigen::Vector4f viewPos = ubo.view * Eigen::Vector4f{current_ori.x(), current_ori.y(), current_ori.z(), 1.f};
        Eigen::Vector4f ndcPos = ubo.projection * viewPos;
        int x = int((ndcPos.x()/ndcPos.w()+1.f)*float(ubo.width)/2.f);
        int y = int((ndcPos.y()/ndcPos.w()+1.f)*float(ubo.height)/2.f);
        float ori_depth = -viewPos.z() / viewPos.w();
        float min_scene_depth = -(*input.gbuffers->position_buffer)(y, x, ColorBit::A);
        if(ori_depth > min_scene_depth + 1e-2)
        {
            hitPos = current_ori;
            return true;
        }
        current_ori += step;
    }

    return false;
}
inline bool rayMarchAcc(Eigen::Vector3f ori, Eigen::Vector3f dir, Eigen::Vector3f& hitPos, const Point& input)
{
    float step_size = 0.033f;
    int level = 0;
    float rate = pow(2.f, float(level));
    int max_search_time = 16;
    bool status = false;

    Eigen::Vector3f current_ori = ori;
    Eigen::Vector3f step = (step_size * rate) * dir;
    current_ori += step;
    for(int i=0; i<max_search_time; i++)
    {
        Eigen::Vector4f viewPos = ubo.view * Eigen::Vector4f{current_ori.x(), current_ori.y(), current_ori.z(), 1.f};
        Eigen::Vector4f ndcPos = ubo.projection * viewPos;
        int x = int((ndcPos.x()/ndcPos.w()+1.f)*float(ubo.width)/2.f) / int(rate);
        int y = int((ndcPos.y()/ndcPos.w()+1.f)*float(ubo.height)/2.f) / int(rate);
        float ori_depth = -viewPos.z() / viewPos.w();
        float min_scene_depth = -(*ubo.depth_mipmap->maps()[level])(y, x, ColorBit::A);
        if(ori_depth > min_scene_depth + 1e-2)
        {
            level--;
            rate = pow(2.f, float(level));
            step = (-step_size * rate) * dir;
            status = true;
        }
        else
        {
            level++;
            level = std::min(level, 8);
            rate = pow(2.f, float(level));
            step = (step_size * rate) * dir;
        }
        if(level<0)
        {
            hitPos = current_ori;
            return true;
        }
        current_ori += step;
    }

    if(status)
    {
        hitPos = current_ori;
        return true;
    }
    return false;
}

inline Eigen::Vector3f point_FragmentShader(const Point& input)
{
    return Eigen::Vector3f{ 1.f, 1.f, 1.f };
}

inline Eigen::Vector3f triangle_FragmentShader(const Point& input)
{
    float visibility = calVisibility(ubo.lights, input, "pcf");
    Eigen::Vector3f color = {1.f, 1.f, 1.f};
    // float cos_a = ubo.faceNormal.dot(ubo.lightDir);
    float cos_a = input.v.normal.normalized().dot(-ubo.lightDir);
    if(cos_a < 0.f)
        return {0.f, 0.f, 0.f};
    return visibility * cos_a * color;
}

inline Eigen::Vector3f normal_FragmentShader(const Point& input)
{
    Eigen::Vector3f color = input.attrs.normal.normalized();
    return color;
}

inline Eigen::Vector3f depth_FragmentShader(const Point& input)
{
    // float gray_value =  input.attrs.position.z();   // no-linear
    float gray_value =  input.z;     // linear
    // std::cout << gray_value << '\n';

    return {gray_value, gray_value, gray_value};
}
inline Eigen::Vector3f shadowmap_FragmentShader(const Point& input)
{
    float visibility = calVisibility(ubo.lights, input, "hard");
    return {visibility, visibility, visibility};
}
inline Eigen::Vector3f gbuffer_FragmentShader(const Point& input)
{
    float pos_x = (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::B);
    float pos_y = (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::G);
    float pos_z = (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::R);
    Eigen::Vector3f pos_color = { pos_x, pos_y, pos_z };
    float depth = (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::A);
    // std::cout << depth << '\n';
    Eigen::Vector3f depth_color = -Eigen::Vector3f{ depth, depth, depth } / 100.f;
    float nor_x = (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::B);
    float nor_y = (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::G);
    float nor_z = (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::R);
    Eigen::Vector3f nor_color = { nor_x, nor_y, nor_z };
    float alb_x = (*input.gbuffers->albedo_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::B);
    float alb_y = (*input.gbuffers->albedo_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::G);
    float alb_z = (*input.gbuffers->albedo_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::R);
    Eigen::Vector3f alb_color = { alb_x, alb_y, alb_z };
    return depth_color;
}
inline Eigen::Vector3f mipmap_FragmentShader(const Point& input)
{
    if(!ubo.depth_mipmap)
        return {0.f, 0.f, 0.f};
    int level = 3;
    int x = input.screen_pos.x() / int(pow(2, level));
    int y = input.screen_pos.y() / int(pow(2, level));
    float pos_x = (*ubo.depth_mipmap->maps()[level])(y, x, ColorBit::B);
    float pos_y = (*ubo.depth_mipmap->maps()[level])(y, x, ColorBit::G);
    float pos_z = (*ubo.depth_mipmap->maps()[level])(y, x, ColorBit::R);
    Eigen::Vector3f pos_color = { pos_x, pos_y, pos_z };
    float depth = (*ubo.depth_mipmap->maps()[level])(y, x, ColorBit::A);
    Eigen::Vector3f depth_color = -Eigen::Vector3f{ depth, depth, depth } / 10.f;

    return depth_color;
}

inline Eigen::Vector3f ssao_FragmentShader(const Point& input)
{
    float occlusion = 0.f;
    float radius = 0.05f;
    float sign = ubo.cameraPos.z()<0.f ? 1.f : -1.f;
    Eigen::Vector3f worldPos = Eigen::Vector3f{ (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::B),
                                               (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::G),
                                               (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::R) };
    Eigen::Vector3f normal = Eigen::Vector3f{ (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::B),
                                               (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::G),
                                               (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::R) };
    for(int i=0; i<ubo.sampleFromHalfSphere.size(); i++)
    {
        Eigen::Vector3f samplePos = input.attrs.TBN * ubo.sampleFromHalfSphere[i].normalized();
        samplePos = worldPos + samplePos.normalized() * radius;
        Eigen::Vector4f offset{samplePos.x(), samplePos.y(), samplePos.z(), 1.f};
        Eigen::Vector4f viewPos = ubo.view * offset;
        Eigen::Vector4f ndcPos = ubo.projection * viewPos;
        int x = int((ndcPos.x()/ndcPos.w()+1.f)*float(ubo.width)/2.f);
        int y = int((ndcPos.y()/ndcPos.w()+1.f)*float(ubo.height)/2.f);
        float depth = -(*input.gbuffers->position_buffer)(y, x, ColorBit::A);
        float sample_depth = -viewPos.z() / viewPos.w();
        // std::cout << sample_depth << ',' << depth << '\n';
        if(sample_depth > depth + 5e-2)
            occlusion += 1.f;
    }
    occlusion /= float(ubo.sampleFromHalfSphere.size());
    float gray = 1.f - occlusion;
    // return {depth, depth, depth};
    return {gray, gray, gray};
}
inline Eigen::Vector3f ssdo_FragmentShader(const Point& input)
{
    Eigen::Vector3f ambient = {0.f, 0.f, 0.f};
    float radius = 0.05f;
    float sign = ubo.cameraPos.z()<0.f ? 1.f : -1.f;
    Eigen::Vector3f worldPos = Eigen::Vector3f{ (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::B),
                                               (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::G),
                                               (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::R) };
    Eigen::Vector3f normal = Eigen::Vector3f{ (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::B),
                                               (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::G),
                                               (*input.gbuffers->normal_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::R) };
    for(int i=0; i<ubo.sampleFromHalfSphere.size(); i++)
    {
        Eigen::Vector3f samplePos = input.attrs.TBN * ubo.sampleFromHalfSphere[i].normalized();
        samplePos = worldPos + samplePos.normalized() * radius;
        Eigen::Vector4f offset{samplePos.x(), samplePos.y(), samplePos.z(), 1.f};
        Eigen::Vector4f viewPos = ubo.view * offset;
        Eigen::Vector4f ndcPos = ubo.projection * viewPos;
        int x = int((ndcPos.x()/ndcPos.w()+1.f)*float(ubo.width)/2.f);
        int y = int((ndcPos.y()/ndcPos.w()+1.f)*float(ubo.height)/2.f);
        float depth = -(*input.gbuffers->position_buffer)(y, x, ColorBit::A);
        float sample_depth = -viewPos.z() / viewPos.w();
        Eigen::Vector3f albedo = Eigen::Vector3f{ (*input.gbuffers->albedo_buffer)(y, x, ColorBit::B),
                                               (*input.gbuffers->albedo_buffer)(y, x, ColorBit::G),
                                               (*input.gbuffers->albedo_buffer)(y, x, ColorBit::R) };
        Eigen::Vector3f l = (samplePos-worldPos).normalized();
        Eigen::Vector3f v = (ubo.cameraPos-worldPos).normalized();
        Eigen::Vector3f h = (l+v).normalized();
        float r2_inverse = 1.f / std::pow((samplePos-worldPos).norm(), 2.f);
        float cos_a = std::max(0.f, normal.dot(h));
        albedo = albedo * cos_a;
        if(sample_depth > depth + 5e-2)
            ambient += albedo;
    }
    ambient /= float(ubo.sampleFromHalfSphere.size());
    return ambient;
}

inline Eigen::Vector3f texture_FragmentShader(const Point& input)
{
    Eigen::Vector3f color = input.colorTexture? input.colorTexture->getColor(input.v.texCoord.x(), input.v.texCoord.y()) : input.v.color;

    return color;
}

inline Eigen::Vector3f albedo_FragmentShader(const Point& input)
{
    // float visibility = calVisibility(ubo.lights, input, "pcss");
    float visibility = 1.f;
    Eigen::Vector3f ka = {0.1f, 0.1f,0.1f};
    Eigen::Vector3f kd = input.colorTexture? input.colorTexture->getColor(input.v.texCoord.x(), input.v.texCoord.y()) : input.v.color;
    Eigen::Vector3f ks = {0.7937f, 0.7937f, 0.7937f};
    Eigen::Vector3f amb_intensity = {1.f, 1.f, 1.f};
    float p = 250.f;

    Eigen::Vector3f La = ka.cwiseProduct(amb_intensity);
    Eigen::Vector3f color = La;
    Eigen::Vector3f n = input.attrs.normal.normalized();
    for(int i=0; i<ubo.lights.size(); i++)
    {
        Eigen::Vector3f l = (ubo.lights[i].pos-input.attrs.modelPos).normalized();
        Eigen::Vector3f v = (ubo.cameraPos-input.attrs.modelPos).normalized();
        Eigen::Vector3f h = (l+v).normalized();
        float cos_a = std::max(0.f, n.dot(h));
        float r = (ubo.lights[i].pos-input.attrs.modelPos).norm();
        float r2_inverse = 1.f/std::pow(r, 2.f);
        

        Eigen::Vector3f Ld = kd.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * cos_a;
        Eigen::Vector3f Ls = ks.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * pow(cos_a, p);
        color += (Ld + Ls);
    }

    return visibility * color;
}

inline Eigen::Vector3f phong_FragmentShader(const Point& input)
{
    float occlusion = calSSAO(input, 0.05f);
    // float occlusion = 0.f;
    float visibility = calVisibility(ubo.lights, input, "pcss");
    // Eigen::Vector3f ka = {0.1f, 0.1f,0.1f};
    Eigen::Vector3f ka =  calSSDO(input, 0.05f);
    Eigen::Vector3f kd = input.colorTexture? input.colorTexture->getColor(input.v.texCoord.x(), input.v.texCoord.y()) : input.v.color;
    Eigen::Vector3f ks = {0.7937f, 0.7937f, 0.7937f};
    Eigen::Vector3f amb_intensity = {0.5f, 0.5f, 0.5f};
    float p = 250.f;

    Eigen::Vector3f La = ka.cwiseProduct(amb_intensity);
    Eigen::Vector3f color = La;
    Eigen::Vector3f n = input.attrs.normal.normalized();
    for(int i=0; i<ubo.lights.size(); i++)
    {
        Eigen::Vector3f l = (ubo.lights[i].pos-input.attrs.modelPos).normalized();
        Eigen::Vector3f v = (ubo.cameraPos-input.attrs.modelPos).normalized();
        Eigen::Vector3f h = (l+v).normalized();
        float r2_inverse = 1.f / std::pow((ubo.lights[i].pos-input.attrs.modelPos).norm(), 2.f);
        float cos_a = std::max(0.f, n.dot(h));

        Eigen::Vector3f Ld = kd.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * cos_a;
        Eigen::Vector3f Ls = ks.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * pow(cos_a, p);
        color += (1.f-occlusion)*(Ld + Ls);
    }

    return visibility * color;
}

inline Eigen::Vector3f mirror_FragmentShader(const Point& input)
{
    auto evalReflectedFunc = [&](Eigen::Vector3f wi, Eigen::Vector3f wo, Eigen::Vector2i screenPos)
    {
        Eigen::Vector3f pos = Eigen::Vector3f{ (*input.gbuffers->position_buffer)(screenPos.y(), screenPos.x(), ColorBit::B),
                                                  (*input.gbuffers->position_buffer)(screenPos.y(), screenPos.x(), ColorBit::G),
                                                  (*input.gbuffers->position_buffer)(screenPos.y(), screenPos.x(), ColorBit::R) };
        Eigen::Vector3f normal = Eigen::Vector3f{ (*input.gbuffers->normal_buffer)(screenPos.y(), screenPos.x(), ColorBit::B),
                                                  (*input.gbuffers->normal_buffer)(screenPos.y(), screenPos.x(), ColorBit::G),
                                                  (*input.gbuffers->normal_buffer)(screenPos.y(), screenPos.x(), ColorBit::R) };
        Eigen::Vector3f reflect_dir = reflect(wo, normal);
        Eigen::Vector3f hitPos = {0.f, 0.f, 0.f};
        if(rayMarch(pos, reflect_dir, hitPos, input))
        {
            Eigen::Vector2i screenHitPos = worldPos2screenPos(hitPos);
            Eigen::Vector3f albedo = Eigen::Vector3f{ (*input.gbuffers->albedo_buffer)(screenHitPos.y(), screenHitPos.x(), ColorBit::B),
                                                      (*input.gbuffers->albedo_buffer)(screenHitPos.y(), screenHitPos.x(), ColorBit::G),
                                                      (*input.gbuffers->albedo_buffer)(screenHitPos.y(), screenHitPos.x(), ColorBit::R) };
            return albedo;
        }
        return Eigen::Vector3f{0.f, 0.f, 0.f};
    };
    Eigen::Vector3f position = Eigen::Vector3f{ (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::B),
                                                (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::G),
                                                (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::R) };
    Eigen::Vector3f wi = (ubo.lights[0].pos-position).normalized();
    Eigen::Vector3f wo = (ubo.cameraPos-position).normalized();
    return evalReflectedFunc(wi, wo, input.screen_pos);
}

inline Eigen::Vector3f ssr_FragmentShader(const Point& input)
{
    float visibility = calVisibility(ubo.lights, input, "pcss");
    float seed = initRand({(input.attrs.ndcPos.x()+1.f)/2.f, (input.attrs.ndcPos.y()+1.f)/2.f});
    auto evalDiffuseFunc = [&](Eigen::Vector3f wi, Eigen::Vector3f wo, Eigen::Vector2i screenPos)
    {   
        Eigen::Vector3f L;
        Eigen::Vector3f albedo = Eigen::Vector3f{ (*input.gbuffers->albedo_buffer)(screenPos.y(), screenPos.x(), ColorBit::B),
                                                  (*input.gbuffers->albedo_buffer)(screenPos.y(), screenPos.x(), ColorBit::G),
                                                  (*input.gbuffers->albedo_buffer)(screenPos.y(), screenPos.x(), ColorBit::R) };
        Eigen::Vector3f normal = Eigen::Vector3f{ (*input.gbuffers->normal_buffer)(screenPos.y(), screenPos.x(), ColorBit::B),
                                                  (*input.gbuffers->normal_buffer)(screenPos.y(), screenPos.x(), ColorBit::G),
                                                  (*input.gbuffers->normal_buffer)(screenPos.y(), screenPos.x(), ColorBit::R) };
        float cos_a = std::max(0.f, normal.dot(wi));
        L = (albedo*INV_PI) * cos_a;
        return L;
    };

    Eigen::Vector3f position = Eigen::Vector3f{ (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::B),
                                                (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::G),
                                                (*input.gbuffers->position_buffer)(input.screen_pos.y(), input.screen_pos.x(), ColorBit::R) };
    
    Eigen::Vector3f L_dir = {0.f, 0.f, 0.f};
    Eigen::Vector3f L_indir = {0.f, 0.f, 0.f};
    for(int k=0; k<ubo.lights.size(); k++)
    {
        Eigen::Vector3f wi = (ubo.lights[k].pos-position).normalized();
        Eigen::Vector3f wo = (ubo.cameraPos-position).normalized();
        float r2_inverse = 1.f / std::pow((ubo.lights[k].pos-position).norm(), 2.f);
        // direct light
        Eigen::Vector3f dir_intensity = r2_inverse * ubo.lights[k].intensity;
        L_dir += visibility * evalDiffuseFunc(wi, wo, input.screen_pos).cwiseProduct(dir_intensity);
        //indirect light
        float pdf = 0.f;
        Eigen::Vector3f subLightPos = {0.f, 0.f, 0.f};
        for(int i=0; i<NUM_SAMPLES; i++)
        {
            Eigen::Vector3f local_dir = sampleHemisphereUniform(seed, pdf).normalized();
            Eigen::Vector3f global_dir = (input.attrs.TBN * local_dir).normalized();
            if(rayMarchAcc(position, global_dir, subLightPos, input))
            {
                r2_inverse = 1.f / std::pow((ubo.lights[k].pos-subLightPos).norm(), 2.f);
                Eigen::Vector3f subLight_wi = (ubo.lights[k].pos-subLightPos).normalized();
                Eigen::Vector2i subLightScreenPos = worldPos2screenPos(subLightPos);
                Eigen::Vector3f indir_intensity = r2_inverse * ubo.lights[k].intensity;
                L_indir += (evalDiffuseFunc(global_dir, wo, input.screen_pos)/pdf).cwiseProduct(evalDiffuseFunc(subLight_wi, -global_dir, subLightScreenPos).cwiseProduct(indir_intensity));
            }
        }
    }
    L_indir /= NUM_SAMPLES;
    

    return  L_dir + L_indir;
}

inline Eigen::Vector3f normalMapping_FragmentShader(const Point& input)
{
    float visibility = calVisibility(ubo.lights, input, "pcf");
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
        Eigen::Vector3f l = (ubo.lights[i].pos-input.attrs.modelPos).normalized();
        Eigen::Vector3f v = (ubo.cameraPos-input.attrs.modelPos).normalized();
        Eigen::Vector3f h = (l+v).normalized();
        float r2_inverse = 1.f / std::pow((ubo.lights[i].pos-input.attrs.modelPos).norm(), 2.f);

        Eigen::Vector3f Ld = kd.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * std::max(0.f, normal.dot(h));
        Eigen::Vector3f Ls = ks.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * pow(std::max(0.f, normal.dot(h)), p);
        color += (Ld + Ls);
    }
    return visibility * color;
}

inline Eigen::Vector3f bumpMapping_FragmentShader(const Point& input)
{
    float visibility = calVisibility(ubo.lights, input, "pcf");
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
        Eigen::Vector3f l = (ubo.lights[i].pos-input.attrs.modelPos).normalized();
        Eigen::Vector3f v = (ubo.cameraPos-input.attrs.modelPos).normalized();
        Eigen::Vector3f h = (l+v).normalized();
        float r2_inverse = 1.f / std::pow((ubo.lights[i].pos-input.attrs.modelPos).norm(), 2.f);

        Eigen::Vector3f Ld = kd.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * std::max(0.f, normal.dot(h));
        Eigen::Vector3f Ls = ks.cwiseProduct(ubo.lights[i].intensity*r2_inverse) * pow(std::max(0.f, normal.dot(h)), p);
        color += (Ld + Ls);
    }
    return visibility * color;
}

inline Eigen::Vector3f displaceMapping_FragmentShader(const Point& input)
{
    float visibility = calVisibility(ubo.lights, input, "pcf");
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
    Eigen::Vector3f position = input.attrs.ndcPos + kn * normal * input.normalTexture->getColor(u_, v_).norm();

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

