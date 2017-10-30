#ifndef VEHICLE_H
#define VEHICLE_H
#include <list>
#include <vector>

using namespace std;

namespace PathPlanner
{
    class Vehicle 
    {
        int _id;
        double _x;
        double _y;
        double _v;
        double _s;
        double _d;
        double _yaw;
        int _lane;
        vector<double> _previous_x;
        vector<double> _previous_y;
        double _end_path_s;
        double _end_path_d;

        // using current d position line will be calculated
        // -1 - Failed to detect line
        // 0 - left line
        // 1 - center line
        // 2 - right line
        void _update_lane() 
        {
            _lane = -1;

            for (int l = 0; l <= 2; l++) 
            {
                if (_d < (2+4*l+2) && _d > (2+4*l-2)) 
                {
                    _lane = l;
                    return;
                }
            }
        }

    public:
        Vehicle() { };

        Vehicle(int id, double x, double y, double v, double s, double d): _id(id), _x(x), _y(y), _v(v), _s(s), _d(d)
        {
            _update_lane();
        };

        Vehicle(const PathPlanner::Vehicle& other ) 
        {
            _id = other._id;
            _x = other._x;
            _y = other._y;
            _s = other._s;
            _yaw = other._yaw;
            _d = other._d;
            _lane = other._lane;
            _previous_x = other._previous_x;
            _previous_y = other._previous_y;
            _end_path_s = other._end_path_s;
            _end_path_d = other._end_path_d;
        };

        void update(double x, double y, double v, double s, double d, double yaw) 
        {
            _x = x;
            _y = y;
            _v = v;
            _s = s;
            _d = d;
            _yaw = yaw;

            _update_lane();
        };

        void set_previous_x_and_y(const vector<double>&previous_x, const vector<double>& previous_y) 
        {
            _previous_x = previous_x;
            _previous_y = previous_y;
        };

        void set_end_path_s_and_d(double end_path_s, double end_path_d) 
        {
            _end_path_s = end_path_s;
            _end_path_d = end_path_d;
        };

        double get_x() const 
        {
            return _x;
        };

        double get_y() const 
        {
            return _y;
        };

        double get_v() const 
        {
            return _v;
        };

        double get_s() const 
        {
            return _s;
        };

        double get_yaw() const 
        {
            return _yaw;
        };

        double get_d() const 
        {
            return _d;
        };

        double get_lane() const 
        {
            return _lane;
        };

        double get_end_path_s() const 
        {
            return _end_path_s;
        };

        double get_end_path_d() const 
        {
            return _end_path_d;
        };

        vector <double> get_previous_x() const 
        {
            return _previous_x;
        };

        vector <double> get_previous_y() const 
        {
            return _previous_y;
        };
    };

    typedef list<PathPlanner::Vehicle> Vehicles;
};

#endif
