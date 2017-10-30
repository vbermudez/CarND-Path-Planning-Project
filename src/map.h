#ifndef MAP_H
#define MAP_H
#include <vector>
#include <string>

using namespace std;

namespace PathPlanner
{ 
    class Map 
    {
    public:
        vector<double> waypoints_x;
        vector<double> waypoints_y;
        vector<double> waypoints_s;
        vector<double> waypoints_dx;
        vector<double> waypoints_dy;

        Map(const string& file_name);

        vector<double> getXY(double s, double d);
    };
};

#endif // MAP_H
