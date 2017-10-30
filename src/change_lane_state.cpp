#include "change_lane_state.h"
#include "keep_lane_state.h"


PathPlanner::NextAction PathPlanner::ChangeLaneState::next_action()
{
    int lane = _planner->car->get_lane();

    if (_desired_lane == lane) 
    {
        _planner->state = PathPlanner::PlannerStatePtr(new PathPlanner::KeepLaneState(_planner));
    }

   return NextAction(_desired_lane, INC_VELOCITY);
}