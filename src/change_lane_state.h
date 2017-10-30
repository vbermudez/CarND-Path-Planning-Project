#ifndef CHANGE_LANE_STATE_H
#define CHANGE_LANE_STATE_H

#include "planner.h"

namespace PathPlanner
{
    class ChangeLaneState: public PlannerState 
    {
        int _desired_lane;

    public:
        ChangeLaneState(int lane, PathPlanner::Planner* planner): PlannerState(planner) 
        {
            _desired_lane = lane;
        }

        ChangeLaneState(const PathPlanner::ChangeLaneState& other): PlannerState(other) 
        {
            _desired_lane = other._desired_lane;
        }

        PathPlanner::NextAction next_action();
    };
};

#endif //CHANGE_LANE_STATE_H
