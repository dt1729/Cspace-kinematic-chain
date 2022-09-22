#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include <vector>
#include "matplotlibcpp.h"
#include "Eigen/Eigen"
#define _USE_MATH_DEFINES
namespace plt = matplotlibcpp;

struct robotCon{
    double l1, l2, l3;
    double theta1, theta2, theta3;

};

void ForwardKinematics();

int main(){
    ForwardKinematics();
    return 0;
}

void ForwardKinematics(){
    robotCon r1;
    std::cout << "Please enter Theta1, Theta2, Theta3 in that order: " << std::endl;
    std::cin >> r1.theta1 >> r1.theta2 >> r1.theta3;
    std::cout << "Please enter l1, l2, l3 in that order: " << std::endl;
    std::cin >> r1.l1 >> r1.l2 >> r1.l3;
    Eigen::Matrix3d R1,R2,R3;
    R1(0,0) = std::cos(r1.theta1); R1(0,1) = -std::sin(r1.theta1); R1(0,2) = 0.0; R1(1,0) = std::sin(r1.theta1); R1(1,1) = std::cos(r1.theta1); R1(1,2) = 0; R1(2,0) = 0; R1(2,1) = 0; R1(2,2) = 0;
    R2(0,0) = std::cos(r1.theta2); R2(0,1) = -std::sin(r1.theta2); R2(0,2) = 0.0; R2(1,0) = std::sin(r1.theta2); R2(1,1) = std::cos(r1.theta2); R2(1,2) = 0; R2(2,0) = 0; R2(2,1) = 0; R2(2,2) = 0;
    R3(0,0) = std::cos(r1.theta3); R3(0,1) = -std::sin(r1.theta3); R3(0,2) = 0.0; R3(1,0) = std::sin(r1.theta3); R3(1,1) = std::cos(r1.theta3); R3(1,2) = 0; R3(2,0) = 0; R3(2,1) = 0; R3(2,2) = 0;
    // // Find your Rotation Matrix
    Eigen::Vector4d T1,T2,T3;
    T1(0) = 0.0; T1(1) = 0.0; T1(2) = 0.0; T1(3) = 1.0; 
    T2(0) = r1.l1; T2(1) = 0.0; T2(2) = 0.0; T2(3) = 1.0; 
    T3(0) = r1.l2; T3(1) = 0.0; T3(2) = 0.0; T3(3) = 1.0; 
    // // Find your translation Vector
    Eigen::Matrix4d Trans1,Trans2,Trans3; // Your Transformation Matrix
    Trans1.setIdentity();Trans2.setIdentity();Trans3.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    Trans1.block<3,3>(0,0) = R1;Trans2.block<3,3>(0,0) = R2;Trans3.block<3,3>(0,0) = R3;
    Trans1.block<4,1>(0,3) = T1;Trans2.block<4,1>(0,3) = T2;Trans3.block<4,1>(0,3) = T3;

    Eigen::Vector4d Al1,Al2,Al3;
    Al1(0) = r1.l1; Al1(1) = 0; Al1(2) = 0; Al1(3) = 1; 
    Al2(0) = r1.l2; Al2(1) = 0; Al2(2) = 0; Al2(3) = 1; 
    Al3(0) = r1.l3; Al3(1) = 0; Al3(2) = 0; Al3(3) = 1; 

    Eigen::Vector4d P1, P2, P3;
    P1 = Trans1*Al1; P2 = Trans1*Trans2*Al2; P3 = Trans1*Trans2*Trans3*Al3;
    plt::named_plot(std::vector<double>{0,P1(0),P2(0),P3(0)}, std::vector<double>{0,P1(1),P2(1),P3(1)},"*-b");   
    plt::grid(true);
    plt::legend();
    plt::xlabel("X axes");
    plt::ylabel("Y axes");
    plt::title("Arm Configuration");
	plt::show();
}