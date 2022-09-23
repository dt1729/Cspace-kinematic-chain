#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include <vector>
#include <unordered_map>
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
}

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

        bool CheckIntersectionWObs(std::pair<double,double> pos);
};

double angle_wrap(double angle);

std::vector<double> IKTwoMemberChain(robotCon r, point p);

std::vector<std::vector<point>> SamplingRobot(std::vector<point> robotPs); // Pass robot points with (0,0) inserted in the array

std::vector<std::vector<double,double>> CSpacePoints(std::vector<std::vector<point>> sampledRobot, std::vector<obstacle> obstacles);
 
// ----------------------------------------------------------------------------------------------------------------//

bool obstacle::CheckIntersectionWObs(point pos){
    // Sum of angles of the point with each vertex point sums to 360 degrees if inside the obstacle
    int n                   = this->obs_points.size();
    float my_sum            = 0;
    bool intersection       = false;
    float prev_min          = INFINITY;
    float dist_from_line    = 0;
    int line_cnt            = 0;

    for(int i = 0; i < this->obs_points.size(); i++){
        // my_sum = sum of all interior angles; interior angles = angle of pos.point vec  - angle of pos.next point vec
        float ang           = std::atan2(this->obs_points[(i+1)%n].second - pos.y, this->obs_points[(i+1)%n].first - pos.x) 
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

std::vector<std::vector<point>> SamplingRobot(std::vector<point> robotPs){
    std::vector<std::vector<point>> sampledR;
    for(int i = 1; i < robotPs.size(); i++){
        std::vector<point> link;
        double lambda = 0.01;
        while(lambda <= 1){
            point temp;
            temp.x = (1-lambda)*robotPs[i-1].x + (lambda)*robotPs[i].x;
            temp.y = (1-lambda)*robotPs[i-1].y + (lambda)*robotPs[i].y;
            link.push_back(temp);
        }
        sampledR.push_back(link);
    }
    return sampledR;
}

std::vector<point> ForwardKinematics(robotCon r1, double theta1, double theta2){
    // Generating rotation matrices
    Eigen::Matrix3d R1,R2;
    R1(0,0) = std::cos(r1.theta1); R1(0,1) = -std::sin(r1.theta1); R1(0,2) = 0.0; 
    R1(1,0) = std::sin(r1.theta1); R1(1,1) =  std::cos(r1.theta1); R1(1,2) = 0.0;
    R1(2,0) = 0; R1(2,1) = 0; R1(2,2) = 0;

    R2(0,0) = std::cos(r1.theta2); R2(0,1) = -std::sin(r1.theta2); R2(0,2) = 0.0;
    R2(1,0) = std::sin(r1.theta2); R2(1,1) =  std::cos(r1.theta2); R2(1,2) = 0.0;
    R2(2,0) = 0; R2(2,1) = 0; R2(2,2) = 0;

    // Generating Trnslation matrices
    Eigen::Vector4d T1,T2;
    T1(0) = 0.0; T1(1) = 0.0; T1(2) = 0.0; T1(3) = 1.0; 
    T2(0) = r1.l1; T2(1) = 0.0; T2(2) = 0.0; T2(3) = 1.0; 

    // Composing Rotation and Translation into a transformation matrix
    Eigen::Matrix4d Trans1,Trans2; 
    Trans1.setIdentity();Trans2.setIdentity();Trans3.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
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
    std::unordered_map<std::vector<double>,std::vector<point>> AnglePoints_map;
    std::unordered_map<std::vector<double>,std::vector<point>> AnglePointsMap_sampled;
    
    double delta_theta = 0.001, theta1 = 0.0, theta2 = 0.0;

    // Generating multiple robot configurations for multiple theta1 and theta2s;
    while(theta1 <= 2*M_PI){
        while(theta2 <= 2*M_PI){
            std::vector<point> ArmConfig = ForwardKinematics(r1, theta1, theta2);
            AnglePoints_map[std::vector<double>{theta1,theta2}] = ArmConfig;
            theta2 += delta_theta;
        }
        theta1 += delta_theta;
    }
    // Sampling points on robot body for each configuration
    for(auto const& x: AnglePoints_map){
        AnglePointsMap_sampled[x.first] = SamplingRobot(x.second);
    }
    // Checking collision of each point with each obstacle
    for(obstacle obs:obstacles){
        for(auto const& x: AnglePointsMap_sampled){
            for(Point p:x.second){
                if(obs.CheckIntersectionWObs(p)){
                    ans.push_back(std::vector<doubles>{x.first[0], x.first[1]});
                    continue;
                }
            }
        }
    }

    return ans;
}