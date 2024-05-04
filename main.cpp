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

const int w = 800;
const int h = 800;
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
    // const std::vector<std::string> obj_paths = {"../../../obj/spot_triangulated_good.obj", "../../../obj/african_head.obj"};
    std::vector<std::string> obj_paths = {"C:/openrenderer/obj/spot_triangulated_good.obj"};


    std::vector<Eigen::Matrix4f> modelMats(obj_paths.size(), Eigen::Matrix4f::Identity());
    std::vector<std::function<Eigen::Vector3f(const vertex_shader_in&, vertex_shader_out&)>> vertexShaders(obj_paths.size(), point_VertexShader);
    std::vector<std::function<Eigen::Vector3f(const Point&)>> fragmentShaders(obj_paths.size(), triangle_FragmentShader);
    // modelMats[0] = translate(Eigen::Vector3f(-1.f, 0.f, 0.f));
    // modelMats[1] = translate(Eigen::Vector3f(1.f, 0.f, 0.f));


    /*1. load the .obj*/
    openrenderer::Loader loader;
    loader.load_obj(obj_paths, vertexShaders, fragmentShaders, modelMats);
    /*2. init SDL*/
    Gui::init(w, h, "openrenderer", SDL_FLIP_VERTICAL);
    Gui::self().create_texture(SDL_PIXELFORMAT_RGB24);
    /*3. init renderer*/
    Render render(w, h, true, true, PixelFormat::RGB888, PixelFormat::RGB888);
    render.init_pipeline(PrimitiveType::TRIANGLE, point_VertexShader, triangle_FragmentShader);
    /*4.init scene*/
    Eigen::Vector3f eyePos(0.f, 0.f, -2.f);
    ubo.init(modelMats, eyePos, w/float(h), Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 1.f, 0.f), 45.f, 0.1f, 10.f, {0.f, 0.f, 1.f});

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
    Gui::self().save_image("C:/openrenderer/result.png", Gui::self().textures[0]);

    Gui::self().quit();
    return 0;    
}




