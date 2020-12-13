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
If you don't want to install the realsense module,just comment the related line in src/CMakelist.txt.
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
- modify the [`OpenCV_DIR`](https://github.com/TouchDeeper/TdLib/blob/dev/src/CMakeLists.txt#L8) to your path


