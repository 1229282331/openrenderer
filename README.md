# 基于CPU模拟的实时软光栅渲染器

### 介绍

​	实时软光栅渲染器基于C++17开发，实现了光栅化渲染管线（顶点装配->顶点着色器->图元组装->光栅化->片段着色器）基本流程，同时借助games101、games202图形学理论实现了**正交/透视投影**、**属性插值与透视投影矫正**、**深度测试**、**Blinn-Phong光照模型**、**法线/凹凸/位移贴图**、**two-pass shadowmap硬阴影/软阴影**（Hard、PCF，PCSS）、**延迟渲染**、**屏幕空间环境光遮蔽SSAO**、**屏幕空间全局光照SSDO**、**屏幕空间光线追踪SSR及其加速结构**（Depth-Mipmap）。该渲染器暂未使用预计算方法，支持动态光源和动态场景，可通过键鼠输入实时改变光源位置、物体位置和摄像机位姿。Openrenderer只是为学习理解硬件渲染管线所做，项目仍需完善补充，未来将持续更新。

### 快速使用

- 测试环境

```
CPU: AMD R5-4800H

操作系统：Windows11/Ubuntu2204

编译器：MinGW-w64(gcc/g++),MSVC,Clang
```

- 编译

```
git clone https://gitee.com/xingheguntangxi/openrenderer.git
cd openrenderer
mkdir build && cd build
cmake ..
cmake --build . -j
```

- 运行

​	编译生成的可执行文件存放在build/bin/目录下，运行前注意更改文件路径。本项目使用.json配置文件加载模型场景，预先提供Marry场景和cornellbox场景。

### 效果

![marry_back](./results/0.png)
![marry_whole](./results//1.png)
![cornellbox_SSR](./results/2.png)
![cornellbox_SSR1](./results/3.png)
