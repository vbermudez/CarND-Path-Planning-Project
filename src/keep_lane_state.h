#ifndef KEEP_LANE_STATE_H
#define KEEP_LANE_STATE_H

#include "planner.h"

namespace PathPlanner
{
    class KeepLaneState: public PlannerState
    {
    public:
        KeepLaneState(PathPlanner::Planner* planner) : PlannerState(planner) { }
        KeepLaneState(const PathPlanner::KeepLaneState& other) : PlannerState(other) { }
        PathPlanner::NextAction next_action();
    };
};

#endif // KEEP_LANE_STATE_H
