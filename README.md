# Ques 2
Implementation of C-Space under translation and rotation for convex obstacle and convex robots,  Forward and inverse kinematics for 3 link arm.

To install Matplotlib.cpp run the following lines

```
sudo apt-get install python-matplotlib python-numpy python3.8-dev 

cd <this_repo>/src
git clone https://github.com/lava/matplotlib-cpp
cd matplotlib-cpp
mkdir build
cd build
cmake .. && make -j4
```

If this runs without any errors that means matplotlibcpp is build. 
Now in this repository I've already placed matplotlibcpp.h in the ```src/```
Next run the following lines to make an a.out file and run the programs
```
cd <this_repo>/src
#to build C-Space code
g++ minkowski_diff.cpp -std=c++11 -I/usr/include/python3.8 -lpython3.8
#to run C-Space builder
./a.out
```
