#ifndef PLANNER_H
#define PLANNER_H

#include <memory>
#include "vehicle.h"
#include "road.h"
#include "map.h"

#define BUFFER_SIZE 35
#define BACK_BUFFER_SIZE 7
#define INC_VELOCITY 0.224
#define FRONT_SAFE_DISTANCE 30
#define MAX_SPEED 49.5

using namespace std;

namespace PathPlanner 
{
    class NextAction 
    {
    public:
        int lane;
        double velocity_change;

        NextAction(int l, double v): lane(l), velocity_change(v) { }

        NextAction(const PathPlanner::NextAction& other) 
        {
            lane = other.lane;
            velocity_change = other.velocity_change;
        }

    };

    class Planner;

    class PlannerState 
    {
    protected:
        PathPlanner::Planner *_planner;

    public:
        PlannerState(PathPlanner::Planner* planner) : _planner(planner) { }

        PlannerState(const PathPlanner::PlannerState& other) 
        {
            _planner = other._planner;
        }

        virtual NextAction next_action() = 0;
    };

    typedef unique_ptr<PathPlanner::PlannerState> PlannerStatePtr;

    class Planner 
    {
        double _velocity = 0;
        vector<double> _next_x_vals;
        vector<double> _next_y_vals;
        
        void _generate_path(int lane);
        void _update_velocity(double change);

    public:
        PathPlanner::Vehicle* car;
        PathPlanner::Road road;
        PathPlanner::PlannerStatePtr state;
        PathPlanner::Map* map;

        bool is_too_close();
        bool is_save_to_change_line(double lane);

        Planner(PathPlanner::Vehicle* car, PathPlanner::Map* map);
        void check(const PathPlanner::Road& road);

        vector<double> get_next_x_vals()  
        {
            return _next_x_vals;
        }

        vector<double> get_next_y_vals() 
        {
            return _next_y_vals;
        }
    };

};

#endif // PLANNER_H
