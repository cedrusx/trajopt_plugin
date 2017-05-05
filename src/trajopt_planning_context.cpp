#include <trajopt_interface_ros/trajopt_planning_context.h>

namespace trajopt_interface_ros
{
TrajoptPlanningContext::TrajoptPlanningContext(const std::string& name, const std::string& group, const robot_model::RobotModelConstPtr& model)
  : planning_interface::PlanningContext(name, group), robot_model_(model)
{
  trajopt_interface_ = TrajoptInterfaceROSPtr(new TrajoptInterfaceROS());
}

TrajoptPlanningContext::~TrajoptPlanningContext()
{
}

bool TrajoptPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  moveit_msgs::MotionPlanResponse res2;
  if (trajopt_interface_->solve(planning_scene_, request_, res2))
  {
    res.trajectory_.resize(1);
    res.trajectory_[0] =
        robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model_, getGroupName()));

    moveit::core::RobotState start_state(robot_model_);
    robot_state::robotStateMsgToRobotState(res2.trajectory_start, start_state);
    res.trajectory_[0]->setRobotTrajectoryMsg(start_state, res2.trajectory);
/*
    trajectory_processing::IterativeParabolicTimeParameterization itp;
    itp.computeTimeStamps(*res.trajectory_[0], request_.max_velocity_scaling_factor,
                          request_.max_acceleration_scaling_factor);
*/
    res.description_.push_back("plan");
    res.processing_time_.push_back(res2.planning_time);
    res.error_code_ = res2.error_code;
    return true;
  }
  else
  {
    res.error_code_ = res2.error_code;
    return false;
  }
}

bool TrajoptPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  planning_interface::MotionPlanDetailedResponse res_detailed;
  bool result = solve(res_detailed);

  res.error_code_ = res_detailed.error_code_;
  res.trajectory_ = res_detailed.trajectory_[0];
  res.planning_time_ = res_detailed.processing_time_[0];

  return result;
}

bool TrajoptPlanningContext::terminate()
{
  // TODO - make interruptible
  return true;
}

void TrajoptPlanningContext::clear()
{
}

} /* namespace trajopt_interface_ros */
