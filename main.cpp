#ifdef USE_SYCL
    #include <sycl/sycl.hpp>
    #include <dpct/dpct.hpp>
    #define EIGEN_USE_SYCL 1
#endif
#include <iostream>
#include <string>
#include <ctime>
#include <chrono>
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

const int w = 1440;
const int h = 1080;
openrenderer::Uniform ubo;

#undef main     // remove the SDL defination for SDL_main: #define main SDL_main
using namespace std;
using namespace openrenderer;


int main(int argc, char* argv[])
{
#ifdef USE_SYCL
    sycl::queue q;
    sycl::device device = q.get_device();
    std::cout << device.get_info<sycl::info::device::name>() << std::endl;
#endif
    // omp_set_num_threads(4);
    // const std::vector<std::string> obj_paths = {"C:/vscode_files/openrenderer/obj/Marry.obj", "C:/vscode_files/openrenderer/obj/floor.obj"};
    std::vector<std::string> obj_paths = {"C:/vscode_files/openrenderer/obj/spot_triangulated_good.obj"};


    std::vector<Eigen::Matrix4f> modelMats(obj_paths.size(), Eigen::Matrix4f::Identity());
    std::vector<std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)>> vertexShaders(obj_paths.size(), nmap_VertexShader);
    std::vector<std::function<Eigen::Vector3f(const Point&)>> fragmentShaders(obj_paths.size(), bumpMapping_FragmentShader);
    Texture niucolorTexture("C:/vscode_files/openrenderer/texture/spot_texture.png");
    Texture niunormalTexture("C:/vscode_files/openrenderer/texture/brickwall_normal.jpg");
    // modelMats[1] = scale(0.2f, 0.2f, 0.2f);
    // fragmentShaders[1] = point_FragmentShader;


    /*1. load the .obj*/
    openrenderer::Loader loader;
    loader.load_obj(obj_paths, vertexShaders, fragmentShaders, {&niucolorTexture}, {&niunormalTexture}, modelMats);
    /*2. init SDL*/
    Gui::init(w, h, "openrenderer", SDL_FLIP_VERTICAL);
    Gui::self().create_texture(SDL_PIXELFORMAT_BGR24);
    /*3. init renderer*/
    Render render(w, h, true, true, PixelFormat::RGB888, PixelFormat::RGB888);
    render.init_pipeline(PrimitiveType::TRIANGLE, ShadeFrequency::GOURAUD, point_VertexShader, texture_FragmentShader);
    /*4.init scene*/
    Eigen::Vector3f eyePos(0.f, 0.f, -1.f);
    std::vector<Light> lights = {
        {{0.f, 20.f, -20.f}, {500, 500, 500}},
        {{20.f, 20.f, 20.f}, {500, 500, 500}},
    };
    ubo.init(w, h, modelMats, eyePos, float(w)/float(h), Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 1.f, 0.f), 75.f/180.f*float(MY_PI), 0.1f, 100.f, {0.f, -1.f, 1.f}, lights);

    //main loop
    SDL_Event event;
    ControlResult state = ControlResult::CONTROL_NONE;
    Gui::self().control.init(&render, &loader);
    while(state!=ControlResult::CONTROL_EXIT)
    {
        SDL_PollEvent(&event);
        state = Gui::self().control.control(event);
        render.drawFrame(loader);
        Gui::self().render_present(Gui::self().textures[0], (void*)render.framebuffers()->color_buffer->buffer, render.width()*render.framebuffers()->color_buffer->pbyte);
        Gui::self().titleFPS();
        
    }
    
    // SDL_Delay(2000);
    Gui::self().save_image("../../../result.png", Gui::self().textures[0]);

    Gui::self().quit();
    return 0;    
}


