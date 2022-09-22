#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include "matplotlibcpp.h"
#define _USE_MATH_DEFINES
namespace plt = matplotlibcpp;

struct Point{
    float x;
    float y;
};

std::vector<Point> Minkowski_diff(std::vector<Point> a, std::vector<Point> b);
std::vector<std::vector<float>> Point2Vec(std::vector<Point> a);
float angle(Point a, Point b);
std::vector<Point> move(std::vector<Point> a, std::vector<Point> b);
std::vector<std::vector<Point>> rotatePoints(std::vector<Point> robot, int partitions);

void part_a();
void part_b();

int main(){
    part_a();
    part_b();
    return 0;
}

void part_b(){
    int partitions = 10;
    std::vector<Point> obstacle             = std::vector<Point>{Point{0,0},Point{1,2},Point{0,2}};
    std::vector<Point> robot                = std::vector<Point>{Point{2,2},Point{3,4},Point{2,4}}; 
    std::vector<std::vector<Point>> rrobot  = rotatePoints(robot, partitions); // Points to be rotated and number of partitions of 360 degrees.
    int count = 0;
    for(std::vector<Point> rot:rrobot){
        std::vector<float> tt;
        std::vector<Point> Cspace_obs           = Minkowski_diff(obstacle, rot);
        std::vector<std::vector<float>> ans1    = Point2Vec(Cspace_obs);
        for(int i = 0; i < ans1[0].size(); i++) tt.push_back((2*M_PI/partitions)*count);
        std::cout << tt.size() << std::endl;
        plt::plot3(ans1[0], ans1[1], tt); 
        count++;
    }
    plt::grid(true);
    plt::legend();
    plt::xlabel("X axes");
    plt::ylabel("Y axes");
    plt::set_zlabel("Theta rotation");
    plt::title("C Space obstacle");
	plt::show();
}

void part_a(){
    std::vector<Point> obstacle             = std::vector<Point>{Point{0,0},Point{1,2},Point{0,2}};
    std::vector<Point> robot                = std::vector<Point>{Point{2,2},Point{3,4},Point{2,4}}; 
    std::vector<Point> Cspace_obs           = Minkowski_diff(obstacle, robot);
    std::vector<std::vector<float>> ans1    = Point2Vec(Cspace_obs);
    std::vector<std::vector<float>> ans2    = Point2Vec(obstacle);
    std::vector<std::vector<float>> ans3    = Point2Vec(robot);
    
    plt::named_plot("CSpace Obstacle", ans1[0], ans1[1],"*b");   
    // plt::named_plot("Trajectory", ans2[0], ans2[1],"*-r");
    // plt::named_plot("Trajectory", ans3[0], ans3[1],"*-g");
    plt::grid(true);
    plt::legend();
    plt::xlabel("X axes");
    plt::ylabel("Y axes");
    plt::title("C Space obstacle");
	plt::show();
}

std::vector<Point> Minkowski_diff(std::vector<Point> a, std::vector<Point> b){
    b               = move(b, a);
    std::vector<Point> minus_b;

    for(Point p: b){
        minus_b.push_back(Point{-p.x, -p.y});
    }

    std::vector<Point> diff_vec;
    int n = a.size()-1; int m = b.size()-1;
    // int i = 0;
    // int j = 0;
    for(int i = 0; i < a.size(); i++){
        for(int j = 0; j < b.size(); j++){
            Point Temp;
            Temp.x = a[i].x + minus_b[j].x;
            Temp.y = a[i].y + minus_b[j].y;
            diff_vec.push_back(Temp);
        }
    }
    // while(i < n+1 && j < m+1){
    //         Point Temp; 
    //         Temp.x = a[i].x + minus_b[j].x;
    //         Temp.y = a[i].y + minus_b[j].y;
    //         std::cout << i << " " << j << std::endl; 
    //         diff_vec.push_back(Temp);
    //         if(angle(a[i+1],a[i]) < angle(b[j+1],b[j])){
    //             i++;
    //         }
    //         else if(angle(a[i+1],a[i]) > angle(b[j+1],b[j])){
    //             j++;
    //         }
    //         else{
    //             i++; j++;
    //         }
    // }

    return diff_vec;
}

std::vector<std::vector<float>> Point2Vec(std::vector<Point> a){
    std::vector<std::vector<float>> ans{std::vector<float>{},std::vector<float>{}};
    for(Point p:a){
        ans[0].push_back(p.x);
        ans[1].push_back(p.y);
    }
    ans[0].push_back(a[0].x);
    ans[1].push_back(a[0].y);

    return ans;
}

float angle(Point a, Point b){
    return std::atan2(b.y - a.y, b.x - a.x);
}

std::vector<Point> move(std::vector<Point> a, std::vector<Point> b){
    float dt = -0.000001;
    float error = 999999;
    while(std::abs(error) >= 0.000001){
        float angle = std::atan2(b[0].y - a[0].y, b[0].x - a[0].x);
        for(int i = 0; i< a.size(); i++){
            a[i].x -= dt*std::cos(angle);
            a[i].y -= dt*std::sin(angle);
        }
        error = std::sqrt(std::pow(a[0].x - b[0].x,2) + std::pow(a[0].y - b[0].y,2));
    }
    return a;
}   


std::vector<std::vector<Point>> rotatePoints(std::vector<Point> robot, int partitions){
    std::vector<std::vector<Point>> ans;
    for(int i = 0; i < partitions; i++){
        std::vector<Point> temp;
        for(Point P:robot){
            Point tempP;
            tempP.x = P.x*std::cos(2*M_PI/partitions * i) - P.y*std::sin(2*M_PI/partitions * i);
            tempP.y = P.x*std::sin(2*M_PI/partitions * i) + P.y*std::cos(2*M_PI/partitions * i);
            temp.push_back(tempP);
        }
        ans.push_back(temp);
    }
    return ans;
}