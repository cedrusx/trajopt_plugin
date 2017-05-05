#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <trajopt_interface_ros/trajopt_interface_ros.h>
#include <trajopt_interface_ros/trajopt_planning_context.h>
#include <boost/shared_ptr.hpp>

#include <pluginlib/class_list_macros.h>

namespace trajopt_interface_ros
{

class TrajoptPlanner : public planning_interface::PlannerManager
{
public:
  bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns)
  {
    std::vector<std::string> groups = model->getJointModelGroupNames();
    ROS_INFO_STREAM("Following groups exist:");
    for (int i = 0; i < groups.size(); i++)
    {
      ROS_INFO("%s", groups[i].c_str());
      planning_contexts_[groups[i]] =
          TrajoptPlanningContextPtr(new TrajoptPlanningContext("trajopt_planning_context", groups[i], model));
    }
    return true;
  }

  bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
  {
    // TODO: Actually respond with something reasonable
    //      capabilities.dummy = false;
    return true;
  }

  planning_interface::PlanningContextPtr getPlanningContext(
    const planning_scene::PlanningSceneConstPtr &planning_scene,
    const moveit_msgs::MotionPlanRequest &req,
    moveit_msgs::MoveItErrorCodes &error_code) const
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    if (req.group_name.empty())
    {
      ROS_ERROR("No group specified to plan for");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    if (!planning_scene)
    {
      ROS_ERROR("No planning scene supplied as input");
      error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }

    planning_contexts_.at(req.group_name)->setPlanningScene(planning_scene);
    planning_contexts_.at(req.group_name)->setMotionPlanRequest(req);
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return planning_contexts_.at(req.group_name);
  }

  std::string getDescription(void) const { return "trajopt"; }

  void getPlanningAlgorithms(std::vector<std::string> &algs) const
  {
    algs.resize(1);
    algs[0] = "trajopt";
  }

  void terminate(void) const
  {
    //TODO - make interruptible
  }

private:
  std::map<std::string, TrajoptPlanningContextPtr> planning_contexts_;
};

} // trajopt_interface_ros

PLUGINLIB_EXPORT_CLASS( trajopt_interface_ros::TrajoptPlanner,
                        planning_interface::PlannerManager);
