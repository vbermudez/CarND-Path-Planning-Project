#include "map.h"
#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "utils.h"

using namespace std;

PathPlanner::Map::Map(const string& file_name)
{

    ifstream in_map_(file_name.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) 
    {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        waypoints_x.push_back(x);
        waypoints_y.push_back(y);
        waypoints_s.push_back(s);
        waypoints_dx.push_back(d_x);
        waypoints_dy.push_back(d_y);
    }
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> PathPlanner::Map::getXY(double s, double d)
{
    int prev_wp = -1;

    while(s > waypoints_s[prev_wp + 1] && (prev_wp < (int)(waypoints_s.size() - 1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1) % waypoints_x.size();
    double heading = atan2((waypoints_y[wp2] - waypoints_y[prev_wp]),(waypoints_x[wp2] - waypoints_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-waypoints_s[prev_wp]);
    double seg_x = waypoints_x[prev_wp] + seg_s * cos(heading);
    double seg_y = waypoints_y[prev_wp] + seg_s * sin(heading);
    double perp_heading = heading-pi() / 2;
    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x,y};
}

