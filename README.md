# Overview
This is a persoal shared library include the commonly used functions in development.
# Getting Started
## Prerequisites
### realsense
- librealsense
- OpenCV
### threadSafeStructure
- None

## Install
If you don't want to install the realsense module,jut comment the related line in src/CMakelist.txt.
```
cmake .. -DCMAKE_INSTALL_PREFIX:=your_path
make 
make install
```
## Usage
- test is used to test the TdLib.
- modify the [`OpenCV_DIR`](https://github.com/TouchDeeper/TdLib/blob/dev/src/CMakeLists.txt#L8) to your path


