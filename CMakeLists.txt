cmake_minimum_required(VERSION 2.8)
project(naolabyrinth)

find_package(qibuild)

# Create a executable named naolabyrinth
# with the source file: main.cpp
qi_create_bin(naolabyrinth "main.cpp")

# Add a simple test:
enable_testing()

qi_use_lib(naolabyrinth ALCOMMON ALPROXIES)
qi_use_lib(naolabyrinth OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_IMGPROC)
