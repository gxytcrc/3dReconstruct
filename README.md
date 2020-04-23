# 3dReconstruct
3D reconstruction system capable of capturing comprehensive dense globally consistentmaps explored using a stereo camera.

# 1. What do I need to build it? #
* Ubuntu
* CMake
* Eigen
* Pangolin
* OpenCV
* Ceres Solver
* PCL

# 2. How do I use it? #
Clone the repository and catkin_make:
```
    git clone https://github.com/zYinji/3dReconstruct,git
    mkdir build
    cd build
    cmake ..
    catkin_make
```
Capture a stereo dataset. Launch it as follows:
```
./3dReconstruct folderName frameNumber
```
