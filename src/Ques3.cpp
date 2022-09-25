#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include <vector>
#include <map>
#include "matplotlibcpp.h"
#include "Eigen/Eigen"
#define _USE_MATH_DEFINES
namespace plt = matplotlibcpp;


struct robotCon{
    double l1, l2;
    double theta1, theta2;
};

struct point{
    double x;
    double y;
};

class obstacle{
    public:
        // The obstacle class is supposed to represent one obstacle 
        // Assumed that points given in acw direction as required by obstacle definitions
        std::vector<std::pair<double,double>> obs_points;

        obstacle(std::vector<std::pair<double,double>> points){
            for(std::pair<double,double> p : points){
                obs_points.push_back(p);
            }
        }

        void plot();
        bool CheckIntersectionWObs(point pos);
};

double angle_wrap(double angle);

std::vector<point> SamplingRobot(std::vector<point> robotPs); // Pass robot points with (0,0) inserted in the array

std::vector<point> ForwardKinematics(robotCon r1, double theta1, double theta2);

std::vector<std::vector<double>> CSpacePoints( robotCon Robot, std::vector<obstacle> obstacles);

void part_A();
void part_B();
void part_C();

// ----------------------------------------------------------------------------------------------------------------//
int main(){
    std::cout << "First assignment questions give output then, user can enter obstacles and robot arm lengths" << std::endl;
    std::cout << "Part A" << std::endl;
    part_A();
    std::cout << "Part B" << std::endl;
    part_B();
    std::cout << "Part C" << std::endl;
    part_C();
    std::vector<obstacle> Union_obs;
    int obs_count = 0;
    std::cout << "Enter obstacle count" << std::endl;
    std::cin >> obs_count;
    int obs_c = 1;
    while(obs_count--){
        std::cout << "Enter total vertices for obstacle " << obs_c << std::endl;
        int vertex_cnt = 0;
        std::cin >> vertex_cnt;
        std::vector<std::pair<double,double>> p0 = {};
        int vv = 1;
        while(vertex_cnt--){
            std::pair<double,double> p;
            std::cout << "Enter x and y coordinate for vertex in that order for vertex " << vv << " separated by a space "<< std::endl;
            std::cin >> p.first >> p.second;
            p0.push_back(p);
            vv++;
        }
        p0.push_back(p0[0]);
        obstacle obs(p0);
        obs.plot();
        Union_obs.push_back(obs);
        obs_c++;
    }

    plt::title("Workspace");
    plt::xlabel("X axes");
    plt::ylabel("Y axes");
    plt::grid(true);
    plt::show();


    robotCon r1;
    r1.l1 = 1; r1.l2 = 1; r1.theta1 = 0; r1.theta2 = 0;

    std::cout << "Enter link length in order of link1, link2 separated by a space" << std::endl;
    std::cin >> r1.l1 >> r1.l2;

    std::vector<std::vector<double>> Cspace;
    

    Cspace = CSpacePoints(r1,Union_obs);
    plt::plot(Cspace[0], Cspace[1],"*b");   
    plt::grid(true);
    plt::xlabel("theta1(radians)");
    plt::ylabel("theta2(radians)");
    plt::title("Two link C space");
	plt::show();


    return 0;
}

void part_A(){
    // Obstacle definition
    std::vector<std::pair<double,double>> p0  = {std::pair<double,double>{0.25,0.25},std::pair<double,double>{0,0.75},std::pair<double,double>{-0.25,0.25},std::pair<double,double>{0.25,0.25}};

    obstacle obs(p0);
    obs.plot();
    plt::title("Workspace");
    plt::xlabel("X axes");
    plt::ylabel("Y axes");
    plt::grid(true);
    plt::show();

    std::vector<obstacle> Union_obstacle;

    Union_obstacle.push_back(obs);

    robotCon r1;
    r1.l1 = 1; r1.l2 = 1; r1.theta1 = 0; r1.theta2 = 0;

    std::vector<std::vector<double>> Cspace;
    

    Cspace = CSpacePoints(r1,Union_obstacle);
    plt::plot(Cspace[0], Cspace[1],"*b");   
    plt::grid(true);
    plt::xlabel("theta1(radians)");
    plt::ylabel("theta2(radians)");
    plt::title("Two link C space");
	plt::show();

}

void part_B(){
        // Obstacle definition
    std::vector<std::pair<double,double>> p1  = {std::pair<double,double>{-0.25,1.1},std::pair<double,double>{-0.25,2},std::pair<double,double>{0.25,2},std::pair<double,double>{0.25,1.1},std::pair<double,double>{-0.25,1.1}};
    std::vector<std::pair<double,double>> p2  = {std::pair<double,double>{-2,-2},std::pair<double,double>{-2,-1.8},std::pair<double,double>{2,-1.8},std::pair<double,double>{2,-2},std::pair<double,double>{-2,-2}};

    obstacle obs1(p1);
    obs1.plot();
    obstacle obs2(p2);
    obs2.plot();
    plt::title("Workspace");
    plt::xlabel("X axes");
    plt::ylabel("Y axes");
    plt::grid(true);
    plt::show();


    std::vector<obstacle> Union_obstacle;

    // Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);
    Union_obstacle.push_back(obs2);

    robotCon r1;
    r1.l1 = 1; r1.l2 = 1; r1.theta1 = 0; r1.theta2 = 0;

    std::vector<std::vector<double>> Cspace;
    

    Cspace = CSpacePoints(r1,Union_obstacle);
    plt::plot(Cspace[0], Cspace[1],"*b");   
    plt::grid(true);
    plt::xlabel("theta1(radians)");
    plt::ylabel("theta2(radians)");
    plt::title("Two link C space");
	plt::show();

}

void part_C(){
    // Obstacle definition
    std::vector<std::pair<double,double>> p1  = {std::pair<double,double>{-0.25,1.1},std::pair<double,double>{-0.25,2},std::pair<double,double>{0.25,2},std::pair<double,double>{0.25,1.1},std::pair<double,double>{-0.25,1.1}};
    std::vector<std::pair<double,double>> p2  = {std::pair<double,double>{-2,-2},std::pair<double,double>{-2,-1.8},std::pair<double,double>{2,-1.8},std::pair<double,double>{2,-2},std::pair<double,double>{-2,-2}};
    std::vector<std::pair<double,double>> p3  = {std::pair<double,double>{-2,-0.5},std::pair<double,double>{-2,-0.3},std::pair<double,double>{2,-0.3},std::pair<double,double>{2,-0.5},std::pair<double,double>{-2,-0.5}};

    obstacle obs1(p1);
    obs1.plot();
    obstacle obs2(p2);
    obs2.plot();
    obstacle obs3(p3);
    obs3.plot();
    plt::grid(true);
    plt::title("Workspace");
    plt::xlabel("X axes");
    plt::ylabel("Y axes");
    plt::show();

    std::vector<obstacle> Union_obstacle;

    Union_obstacle.push_back(obs1);
    Union_obstacle.push_back(obs2);
    Union_obstacle.push_back(obs3);

    robotCon r1;
    r1.l1 = 1; r1.l2 = 1; r1.theta1 = 0; r1.theta2 = 0;

    std::vector<std::vector<double>> Cspace;

    Cspace = CSpacePoints(r1,Union_obstacle);
    plt::plot(Cspace[0], Cspace[1],"*b");   
    plt::grid(true);
    plt::xlabel("theta1(radians)");
    plt::ylabel("theta2(radians)");
    plt::title("Two link C space");
	plt::show();

}

// ----------------------------------------------------------------------------------------------------------------//

bool obstacle::CheckIntersectionWObs(point pos){
    // Sum of angles of the point with each vertex point sums to 360 degrees if inside the obstacle
    int n                    = this->obs_points.size();
    double my_sum            = 0;
    bool intersection        = false;
    double prev_min          = INFINITY;
    double dist_from_line    = 0;
    int line_cnt             = 0;

    for(int i = 0; i < this->obs_points.size(); i++){
        // my_sum = sum of all interior angles; interior angles = angle of pos.point vec  - angle of pos.next point vec
        double ang           = std::atan2(this->obs_points[(i+1)%n].second - pos.y, this->obs_points[(i+1)%n].first - pos.x) 
                            - std::atan2(this->obs_points[i].second - pos.y, this->obs_points[i].first - pos.x);
        ang                 = angle_wrap(ang);
        my_sum              += ang;
    }
    if (std::abs(my_sum)    >= M_PI){
        intersection        = true;
    } 
    return intersection;
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

std::vector<point> SamplingRobot(std::vector<point> robotPs){
    std::vector<point> sampledR;
    for(int i = 1; i < robotPs.size(); i++){
        double lambda = 0.01;
        while(lambda <= 1){
            // std::cout << "sample link " << i << std::endl;
            point temp;
            temp.x = (1-lambda)*robotPs[i-1].x + (lambda)*robotPs[i].x;
            temp.y = (1-lambda)*robotPs[i-1].y + (lambda)*robotPs[i].y;
            sampledR.push_back(temp);
            lambda += 0.01;
        }
    }
    return sampledR;
}

std::vector<point> ForwardKinematics(robotCon r1, double theta1, double theta2){
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
    std::vector<point> ans{origin,p1,p2};
    return ans;
}

std::vector<std::vector<double>> CSpacePoints(robotCon r1, std::vector<obstacle> obstacles){

    std::vector<std::vector<double>> ans;
    std::vector<double> a1,a2,a3,a4;
    std::vector<std::vector<point>> armcfg;
    std::map<std::vector<double>,std::vector<point>> AnglePointsMap_sampled;
    
    double delta_theta = 0.05, theta1 = 0.0, theta2 = 0.0; // To get a refined Cspace lower delta theta further

    // Generating multiple robot configurations for multiple theta1 and theta2s;
    while(theta1 <= 2*M_PI){
        theta2 = 0.0;
        while(theta2 <= 2*M_PI){
            std::vector<point> ArmConfig = ForwardKinematics(r1, theta1, theta2);
            armcfg.push_back(ArmConfig);
            a1.push_back(theta1);
            a2.push_back(theta2);
            theta2 += delta_theta;
        }
        theta1 += delta_theta;
    }
    // Sampling points on robot body for each configuration
    for(int i = 0; i < armcfg.size(); i++){
        AnglePointsMap_sampled[std::vector<double>{a1[i],a2[i]}] = SamplingRobot(armcfg[i]);
    }
    // Checking collision of each point with each obstacle
    int count = 0;
    for(obstacle obs:obstacles){
        for(auto const& x: AnglePointsMap_sampled){
            for(point p:x.second){
                if(obs.CheckIntersectionWObs(p)){
                    a3.push_back(x.first[0]);
                    a4.push_back(x.first[1]);
                    continue;
                }
                count ++;
            }
        }
    }
    ans.push_back(a3);
    ans.push_back(a4);

    return ans;
}
        std::vector<std::pair<double,double>> obs_points;

void obstacle::plot(){
    std::vector<double> x_vals, y_vals;
    for(int i = 0; i < this->obs_points.size(); i++){
        x_vals.push_back(this->obs_points[i].first);
        y_vals.push_back(this->obs_points[i].second);
    }
    plt::named_plot("obstacle ",x_vals,y_vals,"*-");
}