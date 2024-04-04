## GAMES101 作业记录

### Assignment 1

#### 环境搭建

使用平台：Windows + Vscode + MSYS2 + MinGW

#### Eigen 库安装 & 编译

进入[下载地址](https://gitlab.com/libeigen/eigen/-/releases/)进行下载并解压。

```bash
cd /your/path/to/Eigen
mkdir build && cd build
cmake -G "Unix Makefiles" .. # windows 下默认生成 ninja，需要改为生成 makefile
make install -j8
```

然后会自动在 C:/Program Files(x86) 下生成一个名为 `eigen3` 的文件夹。也可以移到自己喜欢的地方，记为 `/your/path/to/eigen3`。

#### opencv 库安装 & 编译

进入[下载地址](https://sourceforge.net/projects/opencvlibrary/files/opencv-win/)进行下载并双击 .exe 文件解压。

```bash
cd /your/path/to/opencv
cd sources
mkdir build && cd build
cmake -G "Unix Makefiles" -D WITH_OPENGL=ON -D ENABLE_CXX11=ON -D WITH_IPP=OFF -D ENABLE_PRECOMPILED_HEADERS=OFF ..
```

接下来用管理员权限运行 `make -j8 && make install -j8`（如果是 Linux 直接 sudo 就好了害）。

会在 `sources/build/` 目录下生成一个名为 `install` 的目录，这就是我们所需要的目录，其他都可以忽略，记为 `your/path/to/opencv`

#### 编译 assignment1

CMakelists 参见分支。

❗ 注意：需要将 `your/path/to/opencv/x64/mingw/bin` 加入系统变量 `PATH`，否则链接阶段会找不到对应的动态库。