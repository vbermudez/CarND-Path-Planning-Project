#ifndef ROAD_H
#define ROAD_H

#include <map>
#include "vehicle.h"

using namespace std;

namespace PathPlanner
{
    class Road
    {
        map<int, PathPlanner::Vehicles> _vehicles_by_lane;

    public:
        Road() { }

        Road(const PathPlanner::Road& other)
        {
            _vehicles_by_lane = other._vehicles_by_lane;
        }

        void clear() 
        {
            _vehicles_by_lane.clear();
        }

        void add(const PathPlanner::Vehicle& v) 
        {
            auto it = _vehicles_by_lane.find(v.get_lane());

            if (it == _vehicles_by_lane.end()) 
            {
                PathPlanner::Vehicles vehicles;
                vehicles.push_back(v);
                _vehicles_by_lane[v.get_lane()] = vehicles;
            }
            else 
            {
                it->second.push_back(v);
            }
        }

        PathPlanner::Vehicles get_vehicles_on_lane(int lane)
        {
            auto it = _vehicles_by_lane.find(lane);

            if (it == _vehicles_by_lane.end())
            {
                return PathPlanner::Vehicles();
            }
            else
            {
                return it->second;
            }
        }

    };
};

#endif // ROAD_H
