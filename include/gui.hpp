#pragma once

#include <iostream>
#include <vector>
#include <ctime>
#include <sstream>
#include "SDL.h"
#include "SDL_image.h"
#include "renderer.hpp"
#include "geometry.hpp"

extern openrenderer::Uniform ubo;

namespace openrenderer{

enum class ControlResult{  
    CONTROL_NONE=0,
    CONTROL_EXIT,
    CONTROL_MOUSEMOTION,
    CONTROL_MOUSEBUTTONDOWN,
    CONTROL_MOUSEBUTTONUP,
};

class GuiControl{
public:
    GuiControl() : m_event({}), m_mousePosX(0), m_mousePosY(0), m_mouseWorldPos(Eigen::Vector3f::Zero()), m_id(0), m_isTranslate(false), m_isRotate(false),  
                    m_oldPos(Eigen::Vector3f::Zero()), m_newPos(Eigen::Vector3f::Zero()), m_render(nullptr), m_loader(nullptr) {  }
    ~GuiControl() { m_render=nullptr;m_loader=nullptr; }
    void init(Render* render, Loader* loader);
    ControlResult control(const SDL_Event& event);


private:
    SDL_Event m_event;
    int m_mousePosX;
    int m_mousePosY;
    Eigen::Vector3f m_mouseWorldPos;
    int m_id;
    bool m_isTranslate;
    bool m_isRotate;
    Eigen::Vector3f m_oldPos;
    Eigen::Vector3f m_newPos;
    
    Render* m_render;
    Loader* m_loader;

};

class Gui{
public:
    static Gui& self() { return *m_self; }
    static bool init(int w, int h, const char* name, SDL_RendererFlip flip=SDL_FLIP_VERTICAL, uint32_t flags=SDL_INIT_VIDEO);
    static void quit();
    void create_texture(SDL_PixelFormatEnum format);
    void render_present(SDL_Texture* texture, const void* pixels, int pitch);
    void save_image(const char*file_name, SDL_Texture* texture, const char* img_type="png", int nbits=32);
    void titleFPS();


    int                       width;
    int                       height; 
    SDL_Window*               window = NULL;
    SDL_Surface*              surface = NULL;
    SDL_Renderer*             render = NULL;
    std::vector<SDL_Texture*> textures;

    GuiControl                control;

private:
    Gui(int w, int h, const char* name, SDL_RendererFlip flip, uint32_t flags);
    ~Gui();

    static Gui* m_self;
    SDL_RendererFlip m_flip;
};




}