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
#include "SDL.h"
#include "SDL_image.h"
// #include "stb_image.h"
#include "Eigen/Eigen"
#include "Eigen/Dense"
#include "loader.hpp"
#include "geometry.hpp"
#include "gui.hpp"
#include "renderer.hpp"
#include "shader.hpp"
#include "texture.hpp"
#include "mipmap.hpp"
// #include "tbb/tbb.h"

const int w = 1080;
const int h = 720;
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

    const std::vector<std::string> obj_paths = {"C:/vscode_files/openrenderer/obj/cube.obj", "C:/vscode_files/openrenderer/obj/Marry.obj", "C:/vscode_files/openrenderer/obj/floor_add.obj"};
    Eigen::Vector3f eyePos(7.f, 7.f, -7.f);
    std::vector<Light> lights = {
        {{0.f, 10.f, 10.f}, {150, 150, 150}, true, Eigen::Matrix4f::Identity(), nullptr},
        // {{0.f, -20.f, -20.f}, {500,500, 500}, false, Eigen::Matrix4f::Identity(), nullptr},
    };
    std::vector<Eigen::Matrix4f> modelMats(obj_paths.size(), Eigen::Matrix4f::Identity());
    std::vector<std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)>> vertexShaders(obj_paths.size(), nmap_VertexShader);
    std::vector<std::function<Eigen::Vector3f(const Point&)>> fragmentShaders(obj_paths.size(), phong_FragmentShader);
    Texture lightcolorTexture("C:/vscode_files/openrenderer/texture/checker.png");
    Texture lightnormalTexture("C:/vscode_files/openrenderer/texture/brickwall_normal.jpg");
    Texture niucolorTexture("C:/vscode_files/openrenderer/texture/MC003_Kozakura_Mari.png");
    Texture niunormalTexture("C:/vscode_files/openrenderer/texture/hmap.jpg");
    Texture floorcolorTexture("C:/vscode_files/openrenderer/texture/checker.png");
    Texture floornormalTexture("C:/vscode_files/openrenderer/texture/brickwall_normal.jpg");
    modelMats[0] = translate(lights[0].pos) * scale(0.1f, 0.1f, 0.1f);
    modelMats[2] = scale(0.15f, 1.f, 0.15f) * translate({0.f, 0.0001f, 0.f});
    fragmentShaders[0] = point_FragmentShader;

    // const std::vector<std::string> obj_paths = {"C:/vscode_files/openrenderer/obj/cornellbox/light.obj", 
    //                                             "C:/vscode_files/openrenderer/obj/cornellbox/floor.obj",
    //                                             "C:/vscode_files/openrenderer/obj/cornellbox/left.obj",
    //                                             "C:/vscode_files/openrenderer/obj/cornellbox/right.obj",};
    // Eigen::Vector3f eyePos(0.f, 0.f, -7.f);
    // std::vector<Light> lights = {
    //     {{228.f, 548.f, 229.5f}, {150, 150, 150}, false, Eigen::Matrix4f::Identity(), nullptr},
    //     // {{0.f, -20.f, -20.f}, {500,500, 500}, false, Eigen::Matrix4f::Identity(), nullptr},
    // };
    // std::vector<Eigen::Matrix4f> modelMats(obj_paths.size(), Eigen::Matrix4f::Identity());
    // std::vector<std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)>> vertexShaders(obj_paths.size(), nmap_VertexShader);
    // std::vector<std::function<Eigen::Vector3f(const Point&)>> fragmentShaders(obj_paths.size(), point_FragmentShader);
    // Texture lightcolorTexture("C:/vscode_files/openrenderer/texture/checker.png");
    // Texture lightnormalTexture("C:/vscode_files/openrenderer/texture/brickwall_normal.jpg");
    // Texture floorcolorTexture("C:/vscode_files/openrenderer/texture/MC003_Kozakura_Mari.png");
    // Texture floornormalTexture("C:/vscode_files/openrenderer/texture/brickwall_normal.jpg");
    // for(int i=0; i<obj_paths.size(); i++)
    //     modelMats[i] = scale(0.01f, 0.01f, 0.01f);
    // modelMats[0] = translate({0.f, -0.01f, 0.f}) * modelMats[0];
    // fragmentShaders[0] = point_FragmentShader;
    // fragmentShaders[1] = floor_FragmentShader;
    // fragmentShaders[2] = left_FragmentShader;
    // fragmentShaders[3] = right_FragmentShader;

    /*1. load the .obj*/
    openrenderer::Loader loader;
    loader.load_obj(obj_paths, vertexShaders, fragmentShaders, {&lightcolorTexture, &niucolorTexture, &floorcolorTexture}, {&lightnormalTexture, &niunormalTexture, &floornormalTexture}, modelMats);
    // loader.load_obj(obj_paths, vertexShaders, fragmentShaders, {&lightcolorTexture, &floorcolorTexture}, {&lightnormalTexture, &floornormalTexture}, modelMats);
    /*2.init scene*/
    ubo.init(w, h, modelMats, eyePos, float(w)/float(h), Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 1.f, 0.f), 75.f/180.f*float(MY_PI), 0.1f, 100.f, {0.f, -1.f, 1.f}, lights);
    loader.objects[0].light = &ubo.lights[0];
    sampleFromHalfSphere(ubo.sampleFromHalfSphere, 10);
    /*3. init SDL*/
    Gui::init(w, h, "openrenderer", SDL_FLIP_VERTICAL);
    Gui::self().create_texture(SDL_PIXELFORMAT_BGR24);
    /*4. init renderer*/
    Render render(w, h, true, true, true, PixelFormat::RGB888, PixelFormat::ARGB8888, PixelFormat::ARGB8888);
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
        for(int i=0; i<100; i++)
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
//     float x[5];
//     memset(x, 0xf1, sizeof(float)*5);
//     std::cout << x[0] << std::endl;
  
// }
  
