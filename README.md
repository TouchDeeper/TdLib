# Overview
This is a persoal shared library include the commonly used functions in development.
# Getting Started
## Prerequisites
- librealsense
- OpenCV
- PCL1.9
- glog
    `sudo apt-get install -y libgoogle-glog-dev`
- sophus 
    intstall the version that has `average.hpp`


## Install
- If you don't want to install the realsense module,just comment the related line in src/CMakelist.txt.
- if OpenCV and PCL is not standard installation
    - modify the `OpenCV_DIR` and `PCL_DIR` to your case
    - you need to add there lib path to `LD_LIBRARY_PATH`. In my case:
     
        `export LD_LIBRARY_PATH=/home/td/slam/install/pcl/lib:$LD_LIBRARY_PATH`.
         
         You can add this code to `~/.bashrc` to make it useful to all terminator.
```
cmake .. -DCMAKE_INSTALL_PREFIX:=your_path
make 
make install
```
an example:
```asm
cmake .. -DCMAKE_INSTALL_PREFIX:=/home/wang/CLionProjects/TdLibrary/install
```

## Usage
- test is used to test the TdLib.


