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
#include "tbb/tbb.h"

const int w = 1440;
const int h = 1024;
openrenderer::Uniform ubo;

#undef main     // remove the SDL defination for SDL_main: #define main SDL_main
using namespace std;
using namespace openrenderer;
using namespace tbb;

double t0 = 0.f;
double t1 = 0.f;

// int main(int argc, char* argv[])
// {
// #ifdef USE_SYCL
//     sycl::queue q;
//     sycl::device device = q.get_device();
//     std::cout << device.get_info<sycl::info::device::name>() << std::endl;
// #endif
//     omp_set_num_threads(6);
//     const std::vector<std::string> obj_paths = {"C:/vscode_files/openrenderer/obj/Marry.obj", "C:/vscode_files/openrenderer/obj/floor.obj"};
//     // std::vector<std::string> obj_paths = {"C:/vscode_files/openrenderer/obj/Marry.obj"};


//     std::vector<Eigen::Matrix4f> modelMats(obj_paths.size(), Eigen::Matrix4f::Identity());
//     std::vector<std::function<Eigen::Vector4f(const vertex_shader_in&, vertex_shader_out&)>> vertexShaders(obj_paths.size(), nmap_VertexShader);
//     std::vector<std::function<Eigen::Vector3f(const Point&)>> fragmentShaders(obj_paths.size(), phong_FragmentShader);
//     Texture niucolorTexture("C:/vscode_files/openrenderer/texture/MC003_Kozakura_Mari.png");
//     Texture niunormalTexture("C:/vscode_files/openrenderer/texture/hmap.jpg");
//     modelMats[1] = scale(0.2f, 0.2f, 0.2f);
//     fragmentShaders[1] = point_FragmentShader;

//     /*1. load the .obj*/
//     openrenderer::Loader loader;
//     loader.load_obj(obj_paths, vertexShaders, fragmentShaders, {&niucolorTexture}, {&niunormalTexture}, modelMats);
//     /*2. init SDL*/
//     Gui::init(w, h, "openrenderer", SDL_FLIP_VERTICAL);
//     Gui::self().create_texture(SDL_PIXELFORMAT_BGR24);
//     /*3. init renderer*/
//     Render render(w, h, true, true, PixelFormat::RGB888, PixelFormat::RGB888);
//     render.init_pipeline(PrimitiveType::TRIANGLE, ShadeFrequency::GOURAUD, point_VertexShader, texture_FragmentShader);
//     /*4.init scene*/
//     Eigen::Vector3f eyePos(2.f, 2.f, -2.f);
//     std::vector<Light> lights = {
//         {{0.f, 20.f, -20.f}, {500, 500, 500}},
//         // {{20.f, 20.f, 20.f}, {500, 500, 500}},
//     };
//     ubo.init(w, h, modelMats, eyePos, float(w)/float(h), Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 1.f, 0.f), 75.f/180.f*float(MY_PI), 0.1f, 100.f, {0.f, -1.f, 1.f}, lights);

//     //main loop
//     SDL_Event event;
//     ControlResult state = ControlResult::CONTROL_NONE;
//     Gui::self().control.init(&render, &loader);
//     while(state!=ControlResult::CONTROL_EXIT)
//     {
//         SDL_PollEvent(&event);
//         state = Gui::self().control.control(event);
//         render.drawFrame(loader);
//         Gui::self().render_present(Gui::self().textures[0], (void*)render.framebuffers()->color_buffer->buffer, render.width()*render.framebuffers()->color_buffer->pbyte);
//         Gui::self().titleFPS();
        
//     }
//     // SDL_Delay(2000);
//     Gui::self().save_image("../../../result.png", Gui::self().textures[0]);

//     Gui::self().quit();
//     printf("triangle() cost %lfms/%.2f%% \n", t0, t0/(t0+t1)*100.f);
//     printf("fragShader() cost %lfms/%.2f%% \n", t1, t1/(t0+t1)*100.f);
//     return 0;    
// }


int main()
{
    auto begin = clock();
    parallel_for(blocked_range<size_t>(0, 20000), [](blocked_range<size_t>& r)
    {
        Eigen::Matrix4f mat = Eigen::Matrix4f::Random();
    });
    // for(int i=0; i<20000; i++)
    // {
    //     Eigen::Matrix4f mat = Eigen::Matrix4f::Random();
    // };
    auto end = clock();
    std::cout << "cost time: " << end-begin << "ms." << std::endl; 
}