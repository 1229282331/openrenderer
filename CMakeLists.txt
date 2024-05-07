cmake_minimum_required(VERSION 3.19 FATAL_ERROR)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/bin")
add_subdirectory(3rdparty)

if(USE_SYCL)
  message("* Using the Intel SYCL compiler.")
  add_definitions(-DUSE_SYCL)
  set(CMAKE_GENERATOR_TOOLSET "Intel(R) oneAPI DPC++ Compiler 2024")
  set(CMAKE_CXX_COMPILER "C:/Program Files (x86)/Intel/oneAPI/compiler/2024.1/bin/dpcpp.exe")
endif()

project(openrenderer VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")  # 对于C语言
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>") # 对于C++语言

# add_definitions("-DNDEBUG")

# ---- Include guards ----
if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(
    FATAL_ERROR
      "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there."
  )
endif()

set(THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/3rdparty")
message(${CMAKE_SOURCE_DIR})



set(EIGEN_INCLUDE ${THIRD_PARTY_DIR}/eigen)
set(SDL2_INCLUDE ${THIRD_PARTY_DIR}/SDL2/include)
set(SDL2image_INCLUDE ${THIRD_PARTY_DIR}/SDL2_image/include)
if(UNIX)
  set(SDL2_LIB 
  ${THIRD_PARTY_DIR}/SDL2/lib/libSDL2.a 
  ${THIRD_PARTY_DIR}/SDL2/lib/libSDL2main.a
  )
  set(SDL2image_LIB 
  ${THIRD_PARTY_DIR}/SDL2_image/lib/libSDL2_image.a
  )
else()
  set(SDL2_LIB 
    ${THIRD_PARTY_DIR}/SDL2/lib/SDL2d.lib 
    ${THIRD_PARTY_DIR}/SDL2/lib/SDL2maind.lib 
    ${THIRD_PARTY_DIR}/SDL2/lib/SDL2-staticd.lib
  )
  set(SDL2image_LIB 
    ${THIRD_PARTY_DIR}/SDL2_image/lib/SDL2_imaged.lib
  )
endif()

set(TARGET_NAME ${CMAKE_PROJECT_NAME})
# 设置VS中的启动项目
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options("/MP")
    set_property(DIRECTORY ${CMAKE_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${TARGET_NAME})
endif()


# 设置项目源代码和头文件
file(GLOB_RECURSE HEADER_FILES "*.h" "*.hpp")
file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")
file(GLOB_RECURSE SHARED_LIB "3rdparty/*.dll")

# 在VS中生成文件树
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${SOURCE_FILES} ${HEADER_FILES} main.cpp)



add_executable(${TARGET_NAME} main.cpp ${SOURCE_FILES} ${HEADER_FILES})

if(USE_SYCL)
  target_compile_options(${TARGET_NAME} PRIVATE -fsycl)
  target_include_directories(${TARGET_NAME} PUBLIC "C:/Program Files (x86)/Intel/oneAPI/compiler/latest/include")
  target_include_directories(${TARGET_NAME} PUBLIC "C:/Program Files (x86)/Intel/oneAPI/dpcpp-ct/latest/include")
else()
  # OpenMP
  FIND_PACKAGE(OpenMP)
  if(OPENMP_FOUND)
      message("OPENMP FOUND")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  endif()
endif()
target_link_libraries(${TARGET_NAME} PUBLIC ${SDL2_LIB})
target_link_libraries(${TARGET_NAME} PUBLIC ${SDL2image_LIB})
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${THIRD_PARTY_DIR}>)
target_include_directories(${TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${EIGEN_INCLUDE}>)
target_include_directories(${TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${SDL2_INCLUDE}>)
target_include_directories(${TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${SDL2image_INCLUDE}>)

file(COPY ${SHARED_LIB} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
file(COPY ${SHARED_LIB} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug)
file(COPY ${SHARED_LIB} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)







