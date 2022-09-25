#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include <vector>
#include <string>
#include "matplotlibcpp.h"
#include "Eigen/Eigen"
#define _USE_MATH_DEFINES
namespace plt = matplotlibcpp;

struct robotCon{
    double l1, l2, l3;
    double theta1, theta2, theta3;
};
struct robotCon2D{
    double l1, l2;
    double theta1, theta2;
};

struct point{
    double x;
    double y;
};

void ForwardKinematics3link();
void ForwardKinematics3link(robotCon r1);
std::vector<point> ForwardKinematics2Link(robotCon2D r1, double theta1, double theta2);

std::vector<double> IKTwoMemberChain(robotCon2D r, point p); // returns two angles

std::vector<double> IKThreeMemberChain(robotCon r, point p); // returns three angles

point CircleIntersection(robotCon r, point p, std::vector<point> armIntersect); // returns intersection point with the circle of the third arm

double angle_wrap(double angle);

int main(){
    bool fk = false;
    std::cout << "Do you want to do forward kinematics? Enter 0(false)/1(true)" << std::endl;
    std::cin >> fk;
    if(fk){
        ForwardKinematics3link();
        return 0;
    }

    bool turn = false;
    std::cout << "Do you want to enter custom IK problem? Enter 0(false)/1(true)" << std::endl;
    std::cin >> turn;
    robotCon r; r.l1 = 1; r.l2 = 0.5; r.l3 = 1;
    point p; p.x = 2; p.y = 0;
    if(turn){
        std::cout << "Please enter l1, l2, l3 in that order separated by a space: " << std::endl;
        std::cin >> r.l1 >> r.l2 >> r.l3;
        std::cout << "Please enter final point coordinates x,y in that order separated by a space: " << std::endl;
        std::cin >> p.x >> p.y;
        if(r.l1+r.l2+r.l3 < std::sqrt(p.x*p.x + p.y*p.y)){
            std::cout << r.l1+r.l2+r.l3 << " " << std::sqrt(p.x*p.x + p.y*p.y)<< std::endl;
            std::cout << "this point is not reachable please retry with different points" << std::endl;
            return 0;
        }
        else if(r.l1+r.l2+r.l3 == std::sqrt(p.x*p.x + p.y*p.y)){
            r.theta1 = std::atan2(p.y,p.x); r.theta2 = 0; r.theta3 = 0;
            ForwardKinematics3link(r);
            return 0;
        }
    }
    std::vector<double> a;
    a = IKThreeMemberChain(r, p);
    r.theta1 = a[0]; r.theta2 = a[1]; r.theta3 = a[2];
    std::cout << "theta1 " << r.theta1 << " theta2 " << r.theta2 << " theta3 " << r.theta3 << std::endl;
    ForwardKinematics3link(r);
    return 0;
}

void ForwardKinematics3link(){
    robotCon r1;
    double a,b,c;
    std::cout << "Please enter Theta1, Theta2, Theta3(in degrees) in that order separated by a space: " << std::endl;
    std::cin >> a >> b >> c;
    r1.theta1 = (M_PI/180)*a; r1.theta2 = (M_PI/180)*b; r1.theta3 = (M_PI/180)*c;

    std::cout << "Please enter l1, l2, l3 in that order separated by a space: " << std::endl;
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
    std::string leg = std::string("End effector x: ") + std::to_string(P3(0)) + std::string(" End effector y: ") + std::to_string(P3(1));
    plt::named_plot(leg,std::vector<double>{0,P1(0),P2(0),P3(0)}, std::vector<double>{0,P1(1),P2(1),P3(1)},"*-b");   
    plt::grid(true);
    plt::xlabel("X axes");
    plt::ylabel("Y axes");
    plt::title(std::string("Arm Configuration;") + std::string(leg));
	plt::show();
}

void ForwardKinematics3link(robotCon r1){
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
    std::string leg = std::string(" theta1 : ") + std::to_string(r1.theta1) + std::string(" theta2 : ") + std::to_string(r1.theta2) + std::string(" theta3 : ") + std::to_string(r1.theta3);
    plt::plot(std::vector<double>{0,P1(0),P2(0),P3(0)}, std::vector<double>{0,P1(1),P2(1),P3(1)},"*-b");   
    plt::grid(true);
    plt::xlabel("X axes");
    plt::ylabel("Y axes");
    plt::title(std::string("Inverse kinematics Arm Configuration angles in radians:") + std::string(leg));
	plt::show();
}

std::vector<point> ForwardKinematics2Link(robotCon2D r1, double theta1, double theta2){
    // Generating rotation matrices
    Eigen::Matrix3d R1,R2;
    R1(0,0) = std::cos(theta1); R1(0,1) = -std::sin(theta1); R1(0,2) = 0.0; 
    R1(1,0) = std::sin(theta1); R1(1,1) =  std::cos(theta1); R1(1,2) = 0.0;
    R1(2,0) = 0; R1(2,1) = 0; R1(2,2) = 0;

    R2(0,0) = std::cos(theta2); R2(0,1) = -std::sin(theta2); R2(0,2) = 0.0;
    R2(1,0) = std::sin(theta2); R2(1,1) =  std::cos(theta2); R2(1,2) = 0.0;
    R2(2,0) = 0; R2(2,1) = 0; R2(2,2) = 0;

    // Generating Trnslation matrices
    Eigen::Vector4d T1,T2;
    T1(0) = 0.0; T1(1) = 0.0; T1(2) = 0.0; T1(3) = 1.0; 
    T2(0) = r1.l1; T2(1) = 0.0; T2(2) = 0.0; T2(3) = 1.0; 

    // Composing Rotation and Translation into a transformation matrix
    Eigen::Matrix4d Trans1,Trans2; 
    Trans1.setIdentity();Trans2.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    Trans1.block<3,3>(0,0) = R1;Trans1.block<4,1>(0,3) = T1;
    Trans2.block<3,3>(0,0) = R2;Trans2.block<4,1>(0,3) = T2;

    Eigen::Vector4d Al1,Al2;
    Al1(0) = r1.l1; Al1(1) = 0; Al1(2) = 0; Al1(3) = 1; 
    Al2(0) = r1.l2; Al2(1) = 0; Al2(2) = 0; Al2(3) = 1; 

    Eigen::Vector4d P1, P2;
    P1 = Trans1*Al1; P2 = Trans1*Trans2*Al2;
    point p1, p2, origin;
    p1.x = P1(0); p1.y = P1(1); p2.x = P2(0); p2.y = P2(1); origin.x = 0.0; origin.y = 0.0;
    std::vector<point> ans{p1,p2};
    return ans;
}

std::vector<double> IKTwoMemberChain(robotCon2D r, point p){ // returns two angles
    double c2, s2, c1, s1;
    double theta1, theta2;

    c2 = (std::pow(p.x,2) + std::pow(p.y,2) - (std::pow(r.l1,2) + std::pow(r.l2,2)))/(2*r.l1*r.l2);
    s2 = std::sqrt(1 - std::pow(c2,2)); // There will be two cases + and - sqrt(1 - c2^2) here we only consider one
    theta2 = std::atan2(s2,c2);
    theta1 = std::atan2(p.y,p.x) - std::atan2(r.l2*s2,r.l1 + r.l2*c2);
    return  std::vector<double>{theta1, theta2};
} 


std::vector<double> IKThreeMemberChain(robotCon r, point p){ // returns three angles
    std::vector<point> armcfg;
    robotCon2D rr; 
    rr.l1 = r.l1; rr.l2 = r.l2; rr.theta1 = 0.0; rr.theta2 = 0.0;
    bool out = false;
    double delta_theta = 0.005, theta1 = 0.0, theta2 = 0.0; // To get a refined Cspace lower delta theta further
    std::vector<point> tt = ForwardKinematics2Link(rr, theta1, theta2);

    if(std::pow(tt[1].y - p.y,2) + std::pow(tt[1].x - p.x,2) <= std::pow(r.l3,2)){
        out = false;
    }
    else{
        out = true;
    }



    // These loops help to find transition state;
    while(theta1 <= 2*M_PI){
        theta2 = 0.0;
        while(theta2 <= 2*M_PI){
            std::vector<point> ArmConfig = ForwardKinematics2Link(rr, theta1, theta2);
            armcfg.push_back(ArmConfig[ArmConfig.size()-1]);
            if(out){  
                if(std::pow(ArmConfig[1].y - p.y,2) + std::pow(ArmConfig[1].x - p.x,2) <= std::pow(r.l3,2)){
                    break;
                }
            }
            else{
                if(std::pow(ArmConfig[1].y - p.y,2) + std::pow(ArmConfig[1].x - p.x,2) > std::pow(r.l3,2)){
                    break;
                } 
            }
            theta2 += delta_theta;
        }
        if(out){            
            if(std::pow(armcfg[armcfg.size()-1].y - p.y,2) + std::pow(armcfg[armcfg.size()-1].x - p.x,2) <= std::pow(r.l3,2)){
                break;
            }
        }
        else{
            if(std::pow(armcfg[armcfg.size()-1].y - p.y,2) + std::pow(armcfg[armcfg.size()-1].x - p.x,2) > std::pow(r.l3,2)){
                break;
            }
        }
        theta1 += delta_theta;
    }


    std::vector<point> armIntersect; armIntersect.push_back(armcfg[armcfg.size() - 1]); armIntersect.push_back(armcfg[armcfg.size() - 2]);
    point s = CircleIntersection(r,p,armIntersect);
    std::cout << s.x << " " <<  s.y << std::endl;
    std::vector<double> FirstTwoAngles;
    FirstTwoAngles = IKTwoMemberChain(rr,s);

    double thirdAngle = std::atan2(p.y - s.y, p.x - s.x) - angle_wrap(FirstTwoAngles[0] + FirstTwoAngles[1]);
    return std::vector<double>{FirstTwoAngles[0], FirstTwoAngles[1], thirdAngle};
} 

point CircleIntersection(robotCon r, point p, std::vector<point> armIntersect){
    std::vector<double> xs = {armIntersect[0].x, armIntersect[1].x};
    std::vector<double> ys = {armIntersect[0].y, armIntersect[1].y};

    double xc = p.x; double yc = p.y;
    double xintr = 0.0, yintr = 0.0;
    double xsmax = 0.0, ysmax = 0.0;
    double xsmin = 0.0, ysmin = 0.0;
    double x1 = 0.0, y1 = 0.0 , x2 = 0.0, y2 = 0.0;
    double d = r.l3, m = 0.0, c = 0.0;

    if(std::abs(xs[0] - xs[1])!=0){
        m = (ys[1] - ys[0])/(xs[1] - xs[0]);
        c = ys[0] - (m)*xs[0];
    }
    else{
        m = 0;
        c = ys[0] - (m)*xs[0];
    }

    if(abs(xs[1] - xs[0]) != 0){
        x1 = (-(m*(c-yc) - xc) + std::sqrt(std::pow(m*(c-yc) - xc,2) - (1+std::pow(m,2))*(std::pow(xc,2) + std::pow((c-yc),2) - std::pow(d,2))))/(1 + std::pow(m,2));
        x2 = (-(m*(c-yc) - xc) - std::sqrt(std::pow(m*(c-yc) - xc,2) - (1+std::pow(m,2))*(std::pow(xc,2) + std::pow((c-yc),2) - std::pow(d,2))))/(1 + std::pow(m,2));

        y1 = m*x1 + c;
        y2 = m*x2 + c;
    }
    else{
        x1 = xs[0];
        x2 = xs[0];

        y1 = std::sqrt(std::pow(d,2) - std::pow((x1-xc),2)) + yc;
        y2 = -std::sqrt(std::pow(d,2) - std::pow((x2-xc),2)) + yc;
    }


    if(xs[0] > xs[1]){
        xsmin = xs[1];
        xsmax = xs[0];
    }
    else{
        xsmin = xs[0];
        xsmax = xs[1];
    }


    if(ys[0] > ys[1]){
        ysmin = ys[1];
        ysmax = ys[0];
    }
    else{
        ysmin = ys[0];
        ysmax = ys[1];
    }

    if((xsmin <= x1 && x1 <= xsmax) && (ysmin <= y1 && y1 <= ysmax)){
        xintr = x1;
        yintr = y1;
    }
    else{
        xintr = x2;
        yintr = y2;
    }
    point ans; ans.x = xintr; ans.y = yintr;
    return ans;
}

double angle_wrap(double angle){
    if(angle > M_PI){
        return -1*(2*M_PI-angle);
    }
    else if(angle < -M_PI){
        return -1*(-2*M_PI-angle);
    }
    return angle;
}