if(NOT TARGET SDL2)
    option(SDL_TEST "" OFF)
    add_subdirectory(SDL2)
endif()

if(NOT TARGET SDL2_image)
    add_subdirectory(SDL2_image)
endif()