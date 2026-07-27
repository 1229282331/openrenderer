# Real-time Software Rasterizer based on CPU Simulation

### Introduction

The Real-time Software Rasterizer is developed using C++ and implements the basic flow of the rasterization rendering pipeline (Vertex Assembly -> Vertex Shader -> Primitive Assembly -> Rasterization -> Fragment Shader). Based on the graphics theory from games101 and games202, it implements **Orthographic/Perspective Projection**, **Attribute Interpolation and Perspective Correcting**, **Depth Testing**, **Blinn-Phong Lighting Model**, **Normal/Bump/Displacement Mapping**, **Two-pass Shadow Map Hard/Soft Shadows** (Hard, PCF, PCSS), **Deferred Rendering**, **Screen Space Ambient Occlusion (SSAO)**, **Screen Space Direct Occlusion (SSDO)**, and **Screen Space Reflections (SSR) with its acceleration structure** (Depth-Mipmap). This renderer does not currently use pre-computation methods and supports dynamic light sources and dynamic scenes; light positions, object positions, and camera poses can be changed in real-time via keyboard and mouse input. Openrenderer is created for the purpose of learning and understanding the hardware rendering pipeline; the project still needs further refinement and will be updated continuously.

### Quick Start
- [bilibili video](https://www.bilibili.com/video/BV1vwYdeREEG/?share_source=copy_web&vd_source=75e3a4981c5fa5c18f79deefa32fb7df)

- Test Environment

```
CPU: AMD R5-4800H

OS: Windows11/Ubuntu2204

Compiler: MinGW-w64(gcc/g++), MSVC, Clang
```

- Compilation

```
git clone https://gitee.com/xingheguntangxi/openrenderer.git
cd openrenderer
mkdir build && cd build
cmake ..
cmake --build . -j
```

- Running

The compiled executable is located in the `build/bin/` directory; please note to change file paths before running. This project uses `.json` configuration files to load model scenes; Marry scene and cornellbox scene are provided in advance.

### Effects

- Two-pass rendering light-view depth map

<div align=center>
	<img src="./results/lightdepth0.png" alt="lightdepth_marry" width="300" />
    <img src="./results/lightdepth1.png" alt="lightdepth_cornellbox" width="300" />
</div>


- Deferred rendering gbuffer (position+depth+normal+albedo)
  <div align=center>
  	<img src="./results/gbuffer_marry_pos.png" alt="gbuffer_marry_pos" width="140" />
  	<img src="./results/gbuffer_marry_depth.png" alt="gbuffer_marry_depth" width="140" />
      <img src="./results/gbuffer_marry_normal.png" alt="gbuffer_marry_normal" width="140" />
      <img src="./results/gbuffer_marry_albedo.png" alt="gbuffer_marry_albedo" width="140" />
  </div>
  
  <div align=center>
    	<img src="./results/gbuffer_cornellbox_pos.png" alt="gbuffer_marry_pos" width="140" />
    	<img src="./results/gbuffer_cornellbox_depth.png" alt="gbuffer_marry_depth" width="140" />
    	<img src="./results/gbuffer_cornellbox_normal.png" alt="gbuffer_marry_normal" width="140" />
    	<img src="./results/gbuffer_cornellbox_albedo.png" alt="gbuffer_marry_albedo" width="140" />
  </div>
- shadowmap (Hard+PCF+PCSS)

<div align=center>
	<img src="./results/marry_shadowmap_hard.png" alt="marry_shadowmap_hard" width="200" />
	<img src="./results/marry_shadowmap_pcf.png" alt="marry_shadowmap_pcf" width="200" />
    <img src="./results/marry_shadowmap_pcss.png" alt="marry_shadowmap_pcss" width="200" />
</div>


- SSR acceleration structure: min-depth mipmap

<div align=center>
	<img src="./results/mipmap0.png" alt="mipmap0" width="100" />
	<img src="./results/mipmap1.png" alt="mipmap1" width="100" />
    <img src="./results/mipmap2.png" alt="mipmap2" width="100" />
    <img src="./results/mipmap3.png" alt="mipmap3" width="100" />
	<img src="./results/mipmap4.png" alt="mipmap4" width="100" />
    <img src="./results/mipmap5.png" alt="mipmap5" width="100" />
</div>


- Screen Space Global Illumination (SSAO+SSDO+SSR)

<div align=center>
	<img src="./results/marry_ssao.png" alt="marry_ssao" width="200" />
	<img src="./results/marry_ssdo.png" alt="marry_ssdo" width="200" />
    <img src="./results/marry_ssr.png" alt="marry_ssr" width="200" />
</div>

<div align=center>
	<img src="./results/cornellbox_ssao.png" alt="cornellbox_ssao" width="200" />
	<img src="./results/cornellbox_ssdo.png" alt="cornellbox_ssdo" width="200" />
    <img src="./results/cornellbox_ssr.png" alt="cornellbox_ssr" width="200" />
</div>


- Normal/Bump Mapping

<div align=center>
	<img src="./results/hmap.jpg" alt="hmap" width="200" />
	<img src="./results/marry_normalmap1.png" alt="marry_normalmap1" width="200" />
    <img src="./results/marry_bumpmap1.png" alt="marry_bumpmap1" width="200" />
</div>

<div align=center>
	<img src="./results/brickwall_normal.jpg" alt="brickwall_normal" width="200" />
	<img src="./results/marry_normalmap2.png" alt="marry_normalmap2" width="200" />
    <img src="./results/marry_bumpmap2.png" alt="marry_bumpmap2" width="200" />
</div>


### Real-time Rendering

#### Basic Blinn-Phong Model

![video load failed](./results/marry_phong.mp4)

#### All Effects Enabled (PCSS Soft Shadows, SSAO, SSDO)

![video load failed](./results/marry_all.mp4)

#### Screen Space Ray Tracing

- Samples = 1

![video load failed](./results/cornellbox_ssr1.mp4)

- Samples = 4

![video load failed](./results/cornellbox_ssr4.mp4)

- Samples = 16

![video load failed](./results/cornellbox_ssr16.mp4)
