#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include <vector>
#include "matplotlibcpp.h"
#include "Eigen/Eigen"
#define _USE_MATH_DEFINES
namespace plt = matplotlibcpp;

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

struct robotCon{
    double l1, l2;
    double theta1, theta2;
};

struct point{
    double x;
    double y;
}

double angle_wrap(double angle);

std::vector<double> IKTwoMemberChain(robotCon r, point p);

std::vector<std::vector<point>> SamplingRobot(std::vector<point> robotPs); // Pass robot points with (0,0) inserted in the array

std::vector<std::vector<double,double>> CSpacePoints(std::vector<std::vector<point>> sampledRobot, std::vector<obstacle> obstacles);
 
// ----------------------------------------------------------------------------------------------------------------//

bool obstacle::CheckIntersectionWObs(std::pair<double,double> pos){
    // Sum of angles of the point with each vertex point sums to 360 degrees if inside the obstacle
    int n                   = this->obs_points.size();
    float my_sum            = 0;
    bool intersection       = false;
    float prev_min          = INFINITY;
    float dist_from_line    = 0;
    int line_cnt            = 0;

    for(int i = 0; i < this->obs_points.size(); i++){
        // my_sum = sum of all interior angles; interior angles = angle of pos.point vec  - angle of pos.next point vec
        float ang           = std::atan2(this->obs_points[(i+1)%n].second - pos.second, this->obs_points[(i+1)%n].first - pos.first) 
                            - std::atan2(this->obs_points[i].second - pos.second, this->obs_points[i].first - pos.first);
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

std::vector<std::vector<double,double>> CSpacePoints(std::vector<std::vector<point>> sampledRobot, std::vector<obstacle> obstacles){
    
}