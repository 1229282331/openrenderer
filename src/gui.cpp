#include "gui.hpp"
#include <string.h>




namespace openrenderer{

Gui* Gui::m_self = nullptr; //single instance

bool Gui::init(int w, int h, const char* name, SDL_RendererFlip flip, uint32_t flags)
{
    m_self = new Gui(w, h, name, flip, flags);
    return m_self->window==NULL ? false : true;
}

void Gui::quit()
{
    delete m_self;
}

void Gui::create_texture(SDL_PixelFormatEnum format)
{
    auto texture = SDL_CreateTexture(render, format, SDL_TEXTUREACCESS_STREAMING, width, height);
    if(!texture)
    {
        printf("[error] SDL texture create error!: %s\n", SDL_GetError()); 
        return;
    }
    textures.push_back(texture);
}

void Gui::render_present(SDL_Texture* texture, const void* pixels, int pitch)
{
    //clear the render screen
    if (SDL_RenderClear(render))
    {
        printf("[error] SDL render screen clear error!: %s\n", SDL_GetError()); 
        return;
    }
    // update the texture data
    if (SDL_UpdateTexture(texture, NULL, pixels, pitch))
    {
        printf("[error] SDL texture update error!: %s\n", SDL_GetError()); 
        return;
    }
    // copy texture to renderer
    SDL_Rect destRect{0, 0, width, height};
    if (SDL_RenderCopy(render, texture, NULL, &destRect))
    {
        printf("[error] SDL texture copy error!: %s\n", SDL_GetError()); 
        return;
    }
    // render present
    SDL_RenderCopyEx(render, texture, NULL, NULL, 0, NULL, m_flip);
    SDL_RenderPresent(render);
}

void Gui::save_image(const char*file_name, SDL_Texture* texture, const char* img_type, int nbits)
{
    SDL_Texture* target = SDL_GetRenderTarget(render);
    SDL_SetRenderTarget(render, texture);
    SDL_Surface* surface = SDL_CreateRGBSurface(0, width, height, nbits, 0, 0, 0, 0);
    SDL_RenderReadPixels(render, NULL, surface->format->format, surface->pixels, surface->pitch);
    
    
    if(strcmp(img_type, "png")==0)
        IMG_SavePNG(surface, file_name);
    else if(strcmp(img_type, "jpg")==0)
        IMG_SaveJPG(surface, file_name, 100);
    else if(strcmp(img_type, "bmp")==0)
        SDL_SaveBMP(surface, file_name);
    else
        printf("[warning] Please choose correct image type: png/jpg/bmp.\n");

    SDL_FreeSurface(surface);
    SDL_SetRenderTarget(render, target);
}

void Gui::titleFPS() 
{
    static clock_t time0 = clock();
    static clock_t time1;
    static clock_t dt;
    static int dframe = 0;
    static std::stringstream info;
    time1 = clock();
    dframe++;
    if ((dt = time1 - time0) >= 1000) 
    {
        info.precision(1);  /*set 1bit precision*/
        info << "openrender" << "    " << std::fixed << dframe*1000 / float(dt) << " FPS";
        SDL_SetWindowTitle(window, info.str().c_str());
        info.str("");   //别忘了在设置完窗口标题后清空所用的stringstream
        time0 = time1;
        dframe = 0;
    }
}

Gui::Gui(int w, int h, const char* name, SDL_RendererFlip flip, uint32_t flags) : width(w), height(h), m_flip(flip)
{                               
    // initial the SDL2
    if(SDL_Init(flags)<0)
    {
        printf("[error] SDL init error!: %s\n", SDL_GetError()); 
        return;
    }
    // initial the SDL_image
    if(!IMG_Init(IMG_INIT_PNG) || !IMG_Init(IMG_INIT_PNG))
    {
        printf("[error] SDL_image init error!: %s\n", SDL_GetError()); 
        return;
    }
    // create window
    window = SDL_CreateWindow(name, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, w, h, SDL_WINDOW_SHOWN);
    if(!window)
    {
        printf("[error] SDL window create error!: %s\n", SDL_GetError()); 
        return;
    }
    // create render
    render = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!render)
    {
        printf("[error] SDL render create error!: %s\n", SDL_GetError()); 
        return;
    }

}

Gui::~Gui()
{
    // destroy texture
    for(int i=0; i<textures.size(); i++)
    {
        SDL_DestroyTexture(textures[i]);
        textures[i] = nullptr;
    }
    // destroy the rendered screen
    if(render)
        SDL_DestroyRenderer(render);
    // destroy the window
    if(window)
        SDL_DestroyWindow(window);
    IMG_Quit();
    SDL_Quit();
}

void GuiControl::init(Render* render, Loader* loader)
{
    m_render = render;
    m_loader = loader;
}


ControlResult GuiControl::control(const SDL_Event& event)
{
    if(event.type==SDL_QUIT)    
        return ControlResult::CONTROL_EXIT;
    else if(event.type==SDL_MOUSEMOTION || event.type==SDL_MOUSEBUTTONDOWN || event.type==SDL_MOUSEBUTTONUP) 
    {
        SDL_GetMouseState(&m_mousePosX, &m_mousePosY);
        switch (event.type)
        {
            case SDL_MOUSEMOTION:		// 鼠标移动
                // std::cout << "[Mouse move]: ( " << mousePosX << ", " << mousePosY << " )" << std::endl;
                if(m_isTranslate)
                {
                    // move the picked object
                    m_newPos = inverse2Dto3D(m_mousePosX, m_render->height()-m_mousePosY, 0.7f, m_render->width(), m_render->height(), Eigen::Matrix4f::Identity(), ubo.view, ubo.projection);
                    Eigen::Vector3f translateVec = m_newPos - m_oldPos;
                    ubo.move_model(m_id, 0.f, Eigen::Vector3f(0.f, 0.f, 0.f), translateVec);
                    m_loader->objects[m_id].bounding_box.p_min += translateVec;
                    m_loader->objects[m_id].bounding_box.p_max += translateVec;
                    m_oldPos = m_newPos;
                }
                else if(m_isRotate)
                {
                    // rotate the picked object
                    m_newPos = inverse2Dto3D(m_mousePosX, m_render->height()-m_mousePosY, 0.7f, m_render->width(), m_render->height(), Eigen::Matrix4f::Identity(), ubo.view, ubo.projection);
                    Eigen::Vector3f rotateVec = m_newPos - m_oldPos;
                    Eigen::Vector3f axis = rotateVec.cross(Eigen::Vector3f(0.f, 0.f, 1.f)).normalized();
                    float angle = 360.f * 0.5f * (abs(rotateVec.x())/abs(m_loader->objects[m_id].bounding_box.p_max.x()-m_loader->objects[m_id].bounding_box.p_min.x()) + abs(rotateVec.y())/abs(m_loader->objects[m_id].bounding_box.p_max.y()-m_loader->objects[m_id].bounding_box.p_min.y()));
                    ubo.move_model(m_id, angle, axis, Eigen::Vector3f::Zero());
                    m_oldPos = m_newPos;
                }
                return ControlResult::CONTROL_MOUSEMOTION;
            case SDL_MOUSEBUTTONDOWN:
                if (event.button.button == SDL_BUTTON_LEFT)
                {
                    m_mouseWorldPos = inverse2Dto3D(m_mousePosX, m_render->height()-m_mousePosY, 0.95f, m_render->width(), m_render->height(), Eigen::Matrix4f::Identity(), ubo.view, ubo.projection);
                    for(int i=0; i<m_loader->objects.size(); i++)
                        if(m_loader->objects[i].bounding_box.intersectP(ubo.cameraPos, (m_mouseWorldPos-ubo.cameraPos).normalized()))
                        {
                            m_id = i;
                            m_isTranslate = true;
                            m_oldPos = inverse2Dto3D(m_mousePosX, m_render->height()-m_mousePosY, 0.7f, m_render->width(), m_render->height(), Eigen::Matrix4f::Identity(), ubo.view, ubo.projection);
                            std::cout << "[Mouse Button Down]: choose the obj to translate." << std::endl;
                            break;
                        }
                }
                else if (event.button.button == SDL_BUTTON_RIGHT)
                {
                    m_mouseWorldPos = inverse2Dto3D(m_mousePosX, m_render->height()-m_mousePosY, 0.95f, m_render->width(), m_render->height(), Eigen::Matrix4f::Identity(), ubo.view, ubo.projection);
                    for(int i=0; i<m_loader->objects.size(); i++)
                        if(m_loader->objects[i].bounding_box.intersectP(ubo.cameraPos, (m_mouseWorldPos-ubo.cameraPos).normalized()))
                        {
                            m_id = i;
                            m_isRotate = true;                            
                            m_oldPos = inverse2Dto3D(m_mousePosX, m_render->height()-m_mousePosY, 0.7f, m_render->width(), m_render->height(), Eigen::Matrix4f::Identity(), ubo.view, ubo.projection);
                            std::cout << "[Mouse Button Down]: choose the obj to rotate." << std::endl;
                            break;
                        }
                }
                return ControlResult::CONTROL_MOUSEBUTTONDOWN;
            case SDL_MOUSEBUTTONUP:		// 鼠标按键松开
                if (event.button.button == SDL_BUTTON_LEFT)
                {
                    if(m_isTranslate)
                    {
                        m_isTranslate = false;
                        std::cout << "[Mouse Button Up]: revoke the obj." << std::endl;
                    }
                }
                else if (event.button.button == SDL_BUTTON_RIGHT)
                {
                    if(m_isRotate)
                    {
                        m_isRotate = false;
                        std::cout << "[Mouse Button Up]: revoke the obj." << std::endl;
                    }
                }
                return ControlResult::CONTROL_MOUSEBUTTONUP;
            default:
                return ControlResult::CONTROL_NONE;
        }
    }
    else
        return ControlResult::CONTROL_NONE;
}


}