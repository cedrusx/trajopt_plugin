#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <trajopt_interface_ros/trajopt_interface_ros.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf/transform_listener.h>

#include <moveit/robot_state/conversions.h>

namespace trajopt_interface_ros
{
MOVEIT_CLASS_FORWARD(TrajoptPlanningContext);

class TrajoptPlanningContext : public planning_interface::PlanningContext
{
public:
  virtual bool solve(planning_interface::MotionPlanResponse& res);
  virtual bool solve(planning_interface::MotionPlanDetailedResponse& res);

  virtual void clear();
  virtual bool terminate();

  TrajoptPlanningContext(const std::string& name, const std::string& group, const robot_model::RobotModelConstPtr& model);

  virtual ~TrajoptPlanningContext();

  void initialize();

private:
  TrajoptInterfaceROSPtr trajopt_interface_;
  moveit::core::RobotModelConstPtr robot_model_;

  boost::shared_ptr<tf::TransformListener> tf_;
};

} /* namespace trajopt_interface_ros */
