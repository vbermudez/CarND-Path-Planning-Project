#include <cstdlib>
#include "planner.h"
#include "keep_lane_state.h"
#include "utils.h"
#include "spline.h"
#include <iostream>

PathPlanner::Planner::Planner(PathPlanner::Vehicle* car, PathPlanner::Map* map): car(car), map(map), _velocity(0)
{
    state = PathPlanner::PlannerStatePtr(new PathPlanner::KeepLaneState(this));
}

void PathPlanner::Planner::_generate_path(int lane) 
{
    double car_s = car->get_s();
    int prev_size = car->get_previous_x().size();

    if (prev_size > 0)
    {
        car_s = car->get_end_path_s();
    }

    vector<double> ptsx;
    vector<double> ptsy;
    double ref_x = car->get_x();
    double ref_y = car->get_y();
    double ref_yaw = deg2rad(car->get_yaw());

    if (prev_size < 2) 
    {
        double prev_car_x = car->get_x() - cos(car->get_yaw());
        double prev_car_y = car->get_y() - sin(car->get_yaw());

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car->get_x());

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car->get_y());
    }
    else 
    {
        ref_x = car->get_previous_x()[prev_size - 1];
        ref_y = car->get_previous_y()[prev_size - 1];

        double ref_x_prev = car->get_previous_x()[prev_size - 2];
        double ref_y_prev = car->get_previous_y()[prev_size - 2];
        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // Generating extra 3 points based on a map
    vector<double> next_wp0 = map->getXY(car_s + 30, (2+4*lane));
    vector<double> next_wp1 = map->getXY(car_s + 60, (2+4*lane));
    vector<double> next_wp2 = map->getXY(car_s + 90, (2+4*lane));

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);
    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // Transform to car orientation
    for (int i = 0; i < ptsx.size(); i++) 
    {
        double shift_x = ptsx[i]-ref_x;
        double shift_y = ptsy[i]-ref_y;
        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0-ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // Generate spline for these points
    tk::spline s;
  
    s.set_points(ptsx, ptsy);
    _next_x_vals.clear();
    _next_y_vals.clear();

    for(int i=0; i < car->get_previous_x().size(); i++) 
    {
        _next_x_vals.push_back(car->get_previous_x()[i]);
        _next_y_vals.push_back(car->get_previous_y()[i]);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
    double x_add_on = 0;

    // Generate new points
    for(int i = 1; i <= 50 - car->get_previous_x().size(); i++) 
    {

        double N = (target_dist / (0.02 * _velocity / 2.24));
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Transform back to map orientation
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
        x_point += ref_x;
        y_point += ref_y;
        _next_x_vals.push_back(x_point);
        _next_y_vals.push_back(y_point);
    }
}

void PathPlanner::Planner::_update_velocity(double change) 
{
    if (change > 0) 
    {
        if (_velocity < MAX_SPEED) 
        {
            _velocity += change;
        }
    }
    else 
    {
        _velocity += change;
    }
}

void PathPlanner::Planner::check(const PathPlanner::Road& road)
{
    this->road = road;

    PathPlanner::NextAction a = state->next_action();

    _update_velocity(a.velocity_change);
    _generate_path(a.lane);
}


bool PathPlanner::Planner::is_too_close() 
{
    double prev_size = car->get_previous_x().size();

    for(auto v: road.get_vehicles_on_lane(car->get_lane())) 
    {
        double check_car_s = v.get_s();

        check_car_s += ((double)prev_size * 0.02 * v.get_v());

        if((check_car_s > car->get_s()) && ((check_car_s-car->get_s()) < BUFFER_SIZE)) 
        {
            return true;
        }
    }

    return false;
}

bool PathPlanner::Planner::is_save_to_change_line(double lane) 
{
    double prev_size = car->get_previous_x().size();

    for (auto v: road.get_vehicles_on_lane(lane)) 
    {
        double distance;

        if (v.get_s() > car->get_s()) 
        {
            distance =  v.get_s() - car->get_s();
            
            if ((v.get_s() - car->get_s()) < FRONT_SAFE_DISTANCE)
            {
                return false;
            }
        }
        else 
        {
            distance = car->get_s() - v.get_s();

            if (v.get_v() > car->get_v()) 
            {
                if (distance < FRONT_SAFE_DISTANCE)
                return false;
            } 
            else if (distance < BACK_BUFFER_SIZE) 
            {
                return false;
            }

        }
    }
    
    return true;
}
