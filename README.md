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
Next run the following lines to make an executable file and run the programs



### Note you can also use ques2 and ques3 executable files present inside src, however matplotlib.cpp should be installed.


# Ques 1
Implementation of C-Space under translation and rotation for convex obstacle and convex robots. Note, while cpp file is present we use python as matplotlibcpps 3D plot does not work properly, the logic of code present in both files is same as that of minkowski difference.

```
cd <this_repo>/src
#to run C-space code question
python3 minkowski_diff.py
```

Next to use the code just follow the instructions step by step as they show up in linux terminal.


# Ques 2
Implementation of Forward and inverse kinematics for 3 link arm. Please compile and run the cpp file using the following lines:

```
cd <this_repo>/src
#to build the code
g++ Ques2.cpp -o ques2 -std=c++11 -I/usr/include/python3.8 -lpython3.8
#to run forward and inverse kinematics of 3 link robot
./ques2
```

Next to use the code just follow the instructions step by step as they show up in linux terminal.

# Ques 3
Implementation of C space visualisation for 2 link(2R) arm. Please compile and run the cpp file using the following lines:
```
cd <this_repo>/src
#to build the code
g++ Ques3.cpp -o ques3 -std=c++11 -I/usr/include/python3.8 -lpython3.8
#to run Cspace visualisation of 2 link(2R) robot
./ques3
```

The code first shows workspace and the obstacles present in the same followed by C-space for all three homework parts, then users can enter their own obstacles, different vertices etc. It is advised to put vertices in anti-clockwise fashion when entering in the terminal for better visualisation.


