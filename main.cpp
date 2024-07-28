#ifdef USE_SYCL
    #include <sycl/sycl.hpp>
    #include <dpct/dpct.hpp>
    #define EIGEN_USE_SYCL 1
#endif
#include <iostream>
#include <string>
#include <ctime>
#include <chrono>
#include "omp.h"
// #include "stb_image.h"
#include "Eigen/Eigen"
#include "Eigen/Dense"
#include "loader.hpp"
#include "geometry.hpp"
#include "gui.hpp"
#include "renderer.hpp"
#include "texture.hpp"
#include "mipmap.hpp"
#include "config.hpp"
// #include "tbb/tbb.h"

// const int w = 600;
// const int h = 600;
openrenderer::Uniform ubo;

#undef main     // remove the SDL defination for SDL_main: #define main SDL_main
using namespace std;
using namespace openrenderer;

double t = 0.f;
double t0 = 0.f;
double t1 = 0.f;
double t2 = 0.f;
double t3 = 0.f;
double t4 = 0.f;

int main(int argc, char* argv[])
{
#ifdef USE_SYCL
    sycl::queue q;
    sycl::device device = q.get_device();
    std::cout << device.get_info<sycl::info::device::name>() << std::endl;
#endif
    // omp_set_num_threads(6);
    omp_set_nested(1);
    bool is_display = true;

    /*0. parse config-file*/
    Config config("C:/vscode_files/openrenderer/cornellbox_scene.json");
    /*1. load the .obj*/
    openrenderer::Loader loader;
    loader.load_obj(config.obj_paths, config.obj_vertexShaders, config.obj_fragmentShaders, config.obj_colorTextures, config.obj_colors, config.obj_normalTextures, config.obj_modelMatrixs);
    /*2.init scene*/
    ubo.init(config.width, config.height, config.obj_modelMatrixs, config.cameraPos, float(config.width)/float(config.height), config.lookAt, config.upDir, config.fovy/180.f*float(MY_PI), config.zNear, config.zFar, {0.f, -1.f, 1.f}, config.lights);
    loader.objects[0].light = &ubo.lights[0];
    sampleFromHalfSphere(ubo.sampleFromHalfSphere, 10);
    /*3. init SDL*/
    Gui::init(config.width, config.height, "openrenderer", SDL_FLIP_VERTICAL);
    Gui::self().create_texture(SDL_PIXELFORMAT_BGR24);
    /*4. init renderer*/
    Render render(config.width, config.height, true, true, config.enableDefferedRender, PixelFormat::RGB888, PixelFormat::ARGB8888, PixelFormat::ARGB8888);
    render.init_pipeline(PrimitiveType::TRIANGLE, ShadeFrequency::GOURAUD, point_VertexShader, texture_FragmentShader);
    
    //main loop
    SDL_Event event;
    ControlResult state = ControlResult::CONTROL_NONE;
    Gui::self().control.init(&render, &loader);
    if(is_display)
    {
        while(state!=ControlResult::CONTROL_EXIT)
        {
            SDL_PollEvent(&event);
            state = Gui::self().control.control(event);
            if(state!=ControlResult::CONTROL_KEYDOWN)
            {
                render.drawFrame(loader);
                // depth2gray(*render.framebuffers()->depth_buffer[0]);
                // Gui::self().render_present(Gui::self().textures[0], (void*)render.framebuffers()->depth_buffer[0]->buffer, render.width()*render.framebuffers()->depth_buffer[0]->pbyte);
                Gui::self().render_present(Gui::self().textures[0], (void*)render.framebuffers()->color_buffer->buffer, render.framebuffers()->color_buffer->width*render.framebuffers()->color_buffer->pbyte);
                Gui::self().titleFPS();
            }
            
        }
    }
    else
    {
        for(int i=0; i<1; i++)
        {
            auto start = std::chrono::high_resolution_clock::now();
            render.drawFrame(loader);
            auto end = std::chrono::high_resolution_clock::now();
            Gui::self().render_present(Gui::self().textures[0], (void*)render.framebuffers()->color_buffer->buffer, render.framebuffers()->color_buffer->width*render.framebuffers()->color_buffer->pbyte);
            t += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()*1e-6;
        }
    }

    Gui::self().save_image("../../../result.png", Gui::self().textures[0]);

    Gui::self().quit();
    printf("[shadow pass] cost %lfms/%.2f%% \n", t0, t0/(t0+t1+t2+t3)*100.f);
    printf("[gbuffer pass] cost %lfms/%.2f%% \n", t1, t1/(t0+t1+t2+t3)*100.f);
    printf("[simple pass] cost %lfms/%.2f%% \n", t2, t2/(t0+t1+t2+t3)*100.f);
    printf("clear() cost %lfms/%.2f%% \n", t3, t3/(t0+t1+t2+t3)*100.f);
    printf("rasterize() cost %lfms/%.2f%% \n", t4, t4/(t2)*100.f);
    printf("frame-time: %.2fms/%.2fms \n", (t0+t1+t2+t3)/100.f, t/100.f);
    return 0;    
}

// int main()
// {
//     Config config("C:/vscode_files/openrenderer/cornellbox_scene.json");

//     Eigen::Vector3f scaleVec;
//     Eigen::Vector3f translateVec;
//     Eigen::Matrix3f rotateMat;    
//     decomposeTRS(config.obj_modelMatrixs[1], scaleVec, translateVec, rotateMat);
//     std::cout << scaleVec << std::endl;

// }

  
