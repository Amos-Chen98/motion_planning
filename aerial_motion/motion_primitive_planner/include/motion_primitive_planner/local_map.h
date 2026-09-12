#pragma once
#include <motion_primitive_planner/root_primitive_generator.h>
#include <rog_map_msgs/LocalMap.h>

namespace motion_primitive_planner {
void validateLocalMap(const rog_map_msgs::LocalMap& map, const gcopter_planner::RoutePlannerConfig& config);
std::shared_ptr<const PlanningSceneSnapshot> buildLocalScene(
    const rog_map_msgs::LocalMap& map, const gcopter_planner::RoutePlannerConfig& config);
}
