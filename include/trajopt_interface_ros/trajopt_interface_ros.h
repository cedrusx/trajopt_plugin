#pragma once
#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <openrave-core.h>
#include <openrave/openrave.h>
#include <osgviewer/osgviewer.hpp>
#include <trajopt/problem_description.hpp>
namespace trajopt_interface_ros
{
MOVEIT_CLASS_FORWARD(TrajoptInterfaceROS);
/** @class TrajOptROS */
class TrajoptInterfaceROS
{
public:
  TrajoptInterfaceROS();
  ~TrajoptInterfaceROS();

  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const moveit_msgs::MotionPlanRequest &req,
             moveit_msgs::MotionPlanResponse &res) const;

protected:

  /** @brief TODO: Configure things using the param server */
  void loadParams(void);

  ros::NodeHandle nh_; /// The ROS node handle
  ros::ServiceClient planner_;
  OpenRAVE::EnvironmentBasePtr penv;
  // OpenRAVE::RobotBasePtr robot;
  // OSGViewerPtr viewer;
  bool enableViewer;
};

}

static inline void jointStateToArray(const robot_model::RobotModelConstPtr& model,
                              const sensor_msgs::JointState &joint_state,
                              const std::string& planning_group_name,
                              Eigen::VectorXd& joint_array)
{
// get the jointmodelgroup that we care about
  const robot_model::JointModelGroup* group = model->getJointModelGroup(planning_group_name);
//vector of joints
  std::vector<const robot_model::JointModel*> models = group->getJointModels();
// Loop over joint_state to find the states that match the names
  for(unsigned int i=0; i < joint_state.position.size(); i++)
  {
    for(size_t j = 0; j < models.size(); j++)
    {
      if(models[j]->getName() == joint_state.name[i])
      {
        joint_array(j) = joint_state.position[i];
      }
    }
  }
}
