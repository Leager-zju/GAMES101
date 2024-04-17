# GAMES101 作业记录

## 环境搭建

使用平台：Windows + Vscode + MSYS2 + MinGW

## Eigen 库安装 & 编译

进入[下载地址](https://gitlab.com/libeigen/eigen/-/releases/)进行下载并解压。

```bash
cd /your/path/to/Eigen
mkdir build && cd build
cmake -G "Unix Makefiles" .. # windows 下默认生成 ninja，需要改为生成 makefile
make install -j8
```

然后会自动在 C:/Program Files(x86) 下生成一个名为 `eigen3` 的文件夹。也可以移到自己喜欢的地方，记为 `/your/path/to/eigen3`。

## opencv 库安装 & 编译

进入[下载地址](https://sourceforge.net/projects/opencvlibrary/files/opencv-win/)进行下载并双击 .exe 文件解压。

```bash
cd /your/path/to/opencv
cd sources
mkdir build && cd build
cmake -G "Unix Makefiles" -D WITH_OPENGL=ON -D ENABLE_CXX11=ON -D WITH_IPP=OFF -D ENABLE_PRECOMPILED_HEADERS=OFF ..
```

接下来用管理员权限运行 `make -j8 && make install -j8`（如果是 Linux 直接 sudo 就好了害）。

会在 `sources/build/` 目录下生成一个名为 `install` 的目录，这就是我们所需要的目录，其他都可以忽略，记为 `your/path/to/opencv`

## 编译

CMakelists 参见相应分支。

❗ 注意：需要将 `your/path/to/opencv/x64/mingw/bin` 加入系统变量 `PATH`，否则链接阶段会找不到对应的动态库。

## Assignment 1

### 注意事项

课程中给的 Squish 矩阵推导仅适用于用右手系，即往 $\mathbf{z}$ 负方向看。

而函数 `get_projection_matrix()` 中的参数 `zNear` 与 `zFar` 都是正数。如果直接用 `zNear`/`zFar` 代入 $n$/$f$，就会导致 $\displaystyle x'=\frac{n}{z}x$ 符号不符合预期，使得观测到的三角形与期望值在  $\mathbf{z}$ 轴上偏离了 180°。

如果不修改框架，就要在 `get_projection_matrix` 加入重新定义 $n$/$f$，即

```C++
float n = -zNear;
float f = -zFar;
```

这样能修复三角形颠倒的问题。

## Assignment 2

### MSAA 做法

原本的框架只用一个值记录某个像素点对应的最小深度，如果使用了 MSAA，就需要记录所有采样点的深度。于是将原来的 `depth_buf` 修改为

```C++
std::vector<std::vector<float>> sample_depth_buf;
```

其中 `sample_depth_buf[pixel_index][sample_index]` 记录了对应采样点的最小深度。

为了使得结果更加平滑，还需要记录每个采样点对应的 RGB 值，最后将像素点内所有采样点的 RGB 作个平均值赋给 `frame_buf[]` 即可。

> 这里如果直接根据采样点在三角形内的数量来计算结果，并直接覆盖 `frame_buf[]`，就会出现黑边。

还要注意修改 `clear()` 函数。

### 注意事项

（如果不实现 MSAA）判断一个点是否在三角形内，不能直接用像素坐标——对应了像素的左下角那个点，而不是像素中心。正确的做法是传参给 `insideTriangle()` 时 `x` 和 `y` 各加上 `0.5`。并且 `insideTriangle()` 的参数类型应从 `int` 改为 `float`。

（如果实现 MSAA）一个采样点对应一个小方块，其边长为 $1/\sqrt{\#(\text{MSAA})}$，所以采样点中心应该要加上边长的一半。