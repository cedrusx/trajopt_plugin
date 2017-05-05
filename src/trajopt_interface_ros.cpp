#include <openrave-core.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <trajopt/common.hpp>
#include <sco/optimizers.hpp>
#include <trajopt/rave_utils.hpp>
#include <utils/eigen_conversions.hpp>
#include <trajopt/plot_callback.hpp>

#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <trajopt_interface_ros/trajopt_interface_ros.h>
#include <trajopt_interface_ros/ros_rave_conversions.h>
#include <boost/foreach.hpp>
#include <iostream>


using namespace std;
namespace trajopt_interface_ros
{

TrajoptInterfaceROS::TrajoptInterfaceROS() :
  nh_("~"), enableViewer(false)
{
  OpenRAVE::RaveInitialize();
  penv = OpenRAVE::RaveCreateEnvironment();
  penv->StopSimulation();
  // Load the PR2 by default
  penv->Load("robots/pr2-beta-static.zae") ;

  loadParams();
}

TrajoptInterfaceROS::~TrajoptInterfaceROS()
{
  penv.reset();
  OpenRAVE::RaveDestroy();
}
void TrajoptInterfaceROS::loadParams(void) {
  nh_.param("enable_viewer", enableViewer, false);
}

bool TrajoptInterfaceROS::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                const moveit_msgs::MotionPlanRequest &req,
                                moveit_msgs::MotionPlanResponse &res) const
{
  ros::WallTime start_time = ros::WallTime::now();

  OpenRAVE::EnvironmentBasePtr myEnv = penv->CloneSelf(OpenRAVE::Clone_Bodies);
  OSGViewerPtr myViewer;
  OpenRAVE::RobotBasePtr myRobot = trajopt::GetRobot(*myEnv);
  assert(myRobot);

  const robot_model::JointModelGroup* model_group =
    planning_scene->getRobotModel()->getJointModelGroup(req.group_name);

  OpenRAVE::RobotBase::ManipulatorPtr manip = getManipulatorFromGroup(myRobot, model_group);

  int numJoints = model_group->getJointModels().size();
  int numSteps;
  nh_.param("num_steps", numSteps, 10);
  Eigen::VectorXd initialState(numJoints);
  Eigen::VectorXd goalState(numJoints);
  ROS_INFO("Planning for %d joints", numJoints);

  // Get the robot's initial joint state
  jointStateToArray(planning_scene->getRobotModel(),
                    req.start_state.joint_state,
                    req.group_name,
                    initialState);

  setRaveRobotState(myRobot, req.start_state.joint_state);

  // Handle multi DOF joints' start state (primarily the base)
  sensor_msgs::MultiDOFJointState multiDofJoints = req.start_state.multi_dof_joint_state;
  vector<geometry_msgs::Transform> transforms = multiDofJoints.transforms;
  vector<string> mdJoints = multiDofJoints.joint_names;
  geometry_msgs::Transform worldJointTransform;
  for(size_t i=0; i<mdJoints.size(); i++){
    ROS_INFO_STREAM("Multi DOF Joint: " << mdJoints[i]);
    if(mdJoints[i] == "world_joint"){ // world_joint represents the offset from odom_combined to base_footprint
      worldJointTransform = transforms[i];
      OpenRAVE::RaveVector<double> trans(transforms[i].translation.x, transforms[i].translation.y, transforms[i].translation.z);
      OpenRAVE::RaveVector<double> rot(transforms[i].rotation.w, transforms[i].rotation.x, transforms[i].rotation.y, transforms[i].rotation.z);
      OpenRAVE::Transform t(rot, trans);
      myRobot->SetTransform(t);
    }
  }

  // Sample the goal constraints to get a joint state
  constraint_samplers::ConstraintSamplerPtr csp = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(planning_scene, req.group_name, req.goal_constraints[0]);

  const robot_state::RobotState &ref_state = planning_scene->getCurrentState();
  robot_state::RobotState sampled_state(ref_state);
  csp->sample(sampled_state, ref_state);
  const double *vpp = sampled_state.getVariablePositions();
  vector<double> sampled_vals(vpp, vpp + sampled_state.getVariableCount());
  ROS_INFO("Sampled goal joint states");
  goalState = util::toVectorXd(sampled_vals);

  ///////////////////////////////////////////////////////
  ///// Set up planning problem
  ///////////////////////////////////////////////////////
  trajopt::ProblemConstructionInfo pci(myEnv);

  trajopt::RobotAndDOFPtr rad(new trajopt::RobotAndDOF(myRobot, manip->GetArmIndices(), 0, OpenRAVE::Vector(0,0,1)));
  pci.rad = rad; // trajopt::RADFromName(manip->GetName(), myRobot);

  trajopt::InitInfo initinfo;
  initinfo.type = trajopt::InitInfo::GIVEN_TRAJ;
  initinfo.data = util::toVectorXd(rad->GetDOFValues()).transpose().replicate(numSteps, 1);

  // Linearly interpolate in joint space
  for(int k=0; k<numJoints; k++){
    initinfo.data.col(k) = Eigen::VectorXd::LinSpaced(numSteps, initialState(k), goalState(k));
  }
  ROS_INFO("Init Traj dimensions: %ld x %ld", initinfo.data.rows(), initinfo.data.cols());

  boost::shared_ptr<trajopt::JointVelCostInfo> jvci(new trajopt::JointVelCostInfo());
  int jointVelCoeffs;
  nh_.param("joint_vel_coeffs", jointVelCoeffs, 1);
  jvci->coeffs = trajopt::DblVec(numJoints, jointVelCoeffs);
  jvci->name = "jvel0";

  boost::shared_ptr<trajopt::CollisionCostInfo> cci(new trajopt::CollisionCostInfo());
  cci->first_step = 0;
  cci->last_step = numSteps-1;
  cci->coeffs = trajopt::DblVec(numSteps, 20);
  cci->dist_pen = trajopt::DblVec(numSteps, 0.035);
  cci->continuous = true;
  cci->name = "continuous_collision";

  // Create optimization constraints for each goal constraint
  // combining position and orientation into pose
  // There are also path and trajectory constraints, not sure of the difference - ignoring for now
  // Also not sure about absolute xyz tolerances on orientation constraints
  for(int c = 0; c < req.goal_constraints.size(); c++){

    // Pose constraints
    map<std::string, geometry_msgs::Point> positions;
    vector<moveit_msgs::PositionConstraint> position_constraints = req.goal_constraints[c].position_constraints;
    // This loop is just in case position and orientation constraints aren't matched up in order for each link
    for(int p = 0; p < position_constraints.size(); p++){
      // Might need to convert to world coordinates?
      positions[position_constraints[p].link_name] = position_constraints[p].constraint_region.primitive_poses[0].position;
    }
    vector<moveit_msgs::OrientationConstraint> orientation_constraints = req.goal_constraints[c].orientation_constraints;
    for(int o = 0; o < position_constraints.size(); o++){
      // Might need to convert to world frame?
      geometry_msgs::Point pos = positions[orientation_constraints[o].link_name];
      geometry_msgs::Quaternion quat = orientation_constraints[o].orientation;
      // Construct a pose constraint
      boost::shared_ptr<trajopt::PoseCostInfo> ppci(new trajopt::PoseCostInfo());
      ppci->timestep = numSteps - 1;
      ppci->xyz = Eigen::Vector3d(pos.x, pos.y, pos.z);
      ppci->wxyz = Eigen::Vector4d(quat.w, quat.x, quat.y, quat.z);
      ppci->pos_coeffs = Eigen::Vector3d::Ones(); // TODO: Configure coefficients
      ppci->rot_coeffs = Eigen::Vector3d::Ones();
      ppci->link = myRobot->GetLink(orientation_constraints[o].link_name);
      pci.cnt_infos.push_back(ppci);
    }

    // Joint constraints -  kind of hacky
    sensor_msgs::JointState js;
    js.name.clear();
    js.position.clear();
    vector<moveit_msgs::JointConstraint> joint_constraints = req.goal_constraints[c].joint_constraints;
    for(int jc = 0; jc < joint_constraints.size(); jc++){
      js.name.push_back(joint_constraints[jc].joint_name);
      js.position.push_back(joint_constraints[jc].position);
    }
    if(!joint_constraints.empty()){
      Eigen::VectorXd goalJointConstraints;
      goalJointConstraints.resize(joint_constraints.size());
      jointStateToArray(planning_scene->getRobotModel(), js,
                        req.group_name, goalJointConstraints);
      boost::shared_ptr<trajopt::JointConstraintInfo> jci(new trajopt::JointConstraintInfo());
      jci->vals = util::toDblVec(goalJointConstraints);
      jci->timestep = numSteps - 1;
      jci->name = "joint0";
      pci.cnt_infos.push_back(jci);
    }
  }

  pci.basic_info.n_steps = numSteps;
  pci.basic_info.manip = manip->GetName();
  pci.basic_info.start_fixed = true;

  pci.init_info = initinfo;
  pci.cost_infos.push_back(jvci);
  pci.cost_infos.push_back(cci);

  trajopt::TrajOptProbPtr prob = trajopt::ConstructProblem(pci);

  ROS_INFO("Problem has %d steps and %d dofs", prob->GetNumSteps(), prob->GetNumDOF());
  sco::BasicTrustRegionSQP opt(prob);

  ros::WallTime create_time = ros::WallTime::now();

  // Copy planning scene obstacles into OpenRAVE world
  vector<OpenRAVE::KinBodyPtr> importedBodies = importCollisionWorld(myEnv, planning_scene->getCollisionWorld());
  ROS_INFO("Imported collision world");

  // Why does this not happen in the constructor for opt?
  opt.initialize(trajopt::trajToDblVec(prob->GetInitTraj()));
  ROS_INFO("Gave optimization initial trajectory");

  if(enableViewer){
    ROS_INFO("Viewer enabled");
    myViewer = OSGViewer::GetOrCreate(myEnv);
    myViewer->UpdateSceneData();
 //   nh_.param("plot_decimation", myViewer->m_plotDecimation, 1);
    opt.addCallback(trajopt::PlotCallback(*prob));
  }

  ROS_INFO("Optimization took %f sec to create", (ros::WallTime::now() - create_time).toSec());

  ros::Time before_opt_time = ros::Time::now();
  opt.optimize();
  ros::Duration optduration = ros::Time::now() - before_opt_time;
  ROS_INFO("Optimization actually took %f sec to run", (optduration).toSec());

  create_time = ros::WallTime::now();

  // assume that the trajectory is now optimized, fill in the output structure:
  res.group_name = req.group_name;
  res.trajectory_start = req.start_state;
  res.planning_time = optduration.toSec();
  trajopt::TrajArray finalTraj = trajopt::getTraj(opt.x(), prob->GetVars());
  // fill in joint names:
  // TODO: May need to make sure that joint names are in the same order
  res.trajectory.joint_trajectory.joint_names.resize(numJoints);
  for (size_t i = 0; i < model_group->getJointModels().size(); i++){
    res.trajectory.joint_trajectory.joint_names[i] = model_group->getJointModels()[i]->getName();
  }

  res.trajectory.joint_trajectory.header = req.start_state.joint_state.header; // @TODO this is probably a hack

  // Fill in multi DOF trajectory (base)
  // Currently hardcoded for stationary base position
  // Eventually should write methods to convert from RobotAndDOF and trajectory
  // array to ROS joints
  trajectory_msgs::MultiDOFJointTrajectory& mdjt = res.trajectory.multi_dof_joint_trajectory;
  mdjt.header.frame_id = req.start_state.joint_state.header.frame_id;
  mdjt.joint_names.clear();
  mdjt.joint_names.push_back("world_joint");


  // fill in the entire trajectory
  res.trajectory.joint_trajectory.points.resize(prob->GetNumSteps());
  res.trajectory.multi_dof_joint_trajectory.points.resize(prob->GetNumSteps());
  for (int i=0; i < prob->GetNumSteps(); i++){
    // Set single DOF joints
    res.trajectory.joint_trajectory.points[i].positions.resize(numJoints);
    for (size_t j=0; j < res.trajectory.joint_trajectory.points[i].positions.size(); j++){
      res.trajectory.joint_trajectory.points[i].positions[j] = finalTraj(i,j);
      if(i == finalTraj.rows()-1) {
        ROS_INFO_STREAM("Joint " << j << " " << res.trajectory.joint_trajectory.points[i].positions[j]);
      }
    }
    // Set multi DOF joints (currently just stationary base)
    mdjt.points[i].transforms.push_back(worldJointTransform);

    // Setting invalid timestamps.
    // Further filtering is required to set valid timestamps accounting for velocity and acceleration constraints.
    res.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
  }

  ROS_INFO("Response took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  ROS_INFO("Serviced planning request in %f wall-seconds, trajectory duration is %f", (ros::WallTime::now() - start_time).toSec(), res.trajectory.joint_trajectory.points[finalTraj.rows()-1].time_from_start.toSec());
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  if(enableViewer){
    myEnv->Remove(myViewer);
    myViewer.reset();
  }
  myEnv->Destroy();

  // I assume the OpenRAVE environment and all its objects are deallocated at
  // the end of the function; should verify that it's not leaking memory
  return true;
}

}
