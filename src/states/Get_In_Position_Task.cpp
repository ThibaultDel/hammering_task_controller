#include "Get_In_Position_Task.h"

#include "../HammeringTaskNew.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cmath>
#include <mc_rtc/logging.h>
#include <fstream> // Required for file operations


void Get_In_Position_Task::configure(const mc_rtc::Configuration & config)
{
   _config.load(config);
    // mc_rtc::log::info("Get_In_Position_Task configure function called with config : \n{}", config.dump(true, true));
    
}

void Get_In_Position_Task::start(mc_control::fsm::Controller & ctl_)
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);
  stop = false;
  _total_time_elapsed = 0.0f;
  ctl.impulsive_constraint_flag = false;
  ctl.impact_detected = false;
  ctl._Activation_height = 0.0;
  load_params();
  add_logs(ctl_);
 
  // Add a stop button to the gui
  ctl.gui()->addElement({}, mc_rtc::gui::Button(ctl.stop_hammering_button_name, [this]() { stop = true; }));

  // ------------------------- BSplineTrajectoryTask ----------------------------
  
  // _constr.end_vel.x() = _magic_normal_final_velocity*ctl.nail_normal_vector_world_frame.x();
  // _constr.end_vel.y() = _magic_normal_final_velocity*ctl.nail_normal_vector_world_frame.y();
  // _constr.end_vel.z() = _magic_normal_final_velocity*ctl.nail_normal_vector_world_frame.z();
  _constr.init_vel.x()=ctl._magic_init_vel(0);
  _constr.init_vel.y()=ctl._magic_init_vel(1);
  _constr.init_vel.z()=ctl._magic_init_vel(2);
  
  
  _constr.end_vel = ctl.nail_rot.transpose()*ctl._magic_final_velocity;

  // No need for orientation waypoints so _oriWp is empty
  _oriWp = {};
  
  // The target is the translation of the nail
  _nail_point = ctl._magic_hitting_target;
  _end_point = _nail_point + Eigen::Vector3d(0, 0, 0);
  _posWp = {ctl.robot().frame(ctl.hammer_head_frame_name).position().translation(),_nail_point};

  // _end_point[2] += 0.02;
  // _target = sva::PTransformd(_end_point);
  // _target = sva::PTransformd(Eigen::Quaterniond(0.0f, 0.708, 0.0f, -0.705)) *sva::PTransformd(_end_point);//* sva::PTransformd(Eigen::Vector3d(0.5, 0.2, 1));
  _target = sva::PTransformd(sva::RotX(M_PI)) * sva::PTransformd(sva::RotY(M_PI/2)) * sva::PTransformd(ctl.robots().robot(ctl.nail_robot_name).frame(ctl.nail_frame_name).position().rotation()) *sva::PTransformd(_end_point);//* sva::PTransformd(Eigen::Vector3d(0.5, 0.2, 1));

  // The curve has at least 2 control points : the first one being the initial translation of the frame and the second 
  // being the target, thus the curve is at least of degree 1
  // Curve constraints add 4 control points and we already have the starting point and the final point.
  // Thus, the degree of the BSpline is 5 (the degree is N-1 points).

  mc_rtc::log::info("Adding BSpline task with weight {}", ctl._magic_BSpline_task_weight);
  
  _BSplineVel = std::make_shared<mc_tasks::BSplineTrajectoryTask>(ctl.robot().frame(ctl.hammer_head_frame_name),
                                                                  ctl._magic_BSpline_max_duration,
                                                                  ctl._magic_BSpline_task_stiffness, 
                                                                  ctl._magic_BSpline_task_weight, 
                                                                  _target, 
                                                                  _constr,
                                                                  _posWp, 
                                                                  _oriWp);

  _BSplineVel->setGains(ctl._magic_BSpline_task_stiffness, ctl._magic_BSpline_task_damping);

  _BSplineVel->dimWeight(ctl.dimweights);
  ctl.solver().addTask(_BSplineVel);
  ctl.bspline_active_ = true;

  mc_rtc::log::info("Degree of BSpline : {}", _BSplineVel->spline().get_bezier()->degree());

  // ------------------------- Posture task tunnning ----------------------------

  ctl.getPostureTask(ctl.robot().name())->stiffness(_magic_posture_task_stiffness);
  ctl.getPostureTask(ctl.robot().name())->weight(ctl._magic_posture_task_weight);

  // ------------------------- Gripper task ----------------------------

  // gripper_task->target(sva::PTransformd(sva::RotY(M_PI)) * sva::PTransformd(sva::RotZ(M_PI/2)) * sva::PTransformd(Eigen::Vector3d(0 ,0, 0.025)) * ctl.robot("box").frame("Right").position());

  auto gripper_target = sva::PTransformd(Eigen::Quaterniond(0.0f, 0.708, 0.0f, -0.705)) * sva::PTransformd(_end_point);//sva::PTransformd(Eigen::Vector3d(0.7, 0.5, 1)) * 
  // Test transform task
  gripper_task = std::make_shared<mc_tasks::TransformTask>(ctl.robot().frame(ctl.hammer_head_frame_name), _gripper_task_min_stiffness, _gripper_task_weight);
  // ctl.solver().addTask(gripper_task);
  gripper_task->target(gripper_target);

  Eigen::Vector6d dimweights_grip = gripper_task->dimWeight();

  // ------------------------- VectorOrientationTask ----------------------------

  _vectorOrientationTask = std::make_shared<mc_tasks::VectorOrientationTask>(ctl.robot().frame(ctl.hammer_head_frame_name),
                                                                            ctl.normal_vector_to_align_in_hammerhead_frame
  );
  _vectorOrientationTask->targetVector(-ctl.nail_normal_vector_world_frame);
  _vectorOrientationTask->weight(ctl._magic_vector_orientation_task_weight);
  _vectorOrientationTask->stiffness(ctl._magic_vector_orientation_task_stiffness);
  _vectorOrientationTask->damping(ctl._magic_vector_orientation_task_damping);
  ctl.solver().addTask(_vectorOrientationTask);
  
  mc_rtc::log::info("Mass of the nail = {} kg", ctl.robot(ctl.nail_robot_name).mass());
  //mc_rtc::log::info("solver timestep = {} s", ctl.solver().dt());

  // // Add impulsive torque constraint
  // Eigen::Vector3d normal_nail = ctl.robot(ctl.nail_robot_name).frame(ctl.nail_frame_name).position().rotation().col(2).eval();
  if(ctl.linear_impulsive_torque_ctr_flag){
    ctl.impulseConstraint = std::make_unique<mc_solver::ImpulseConstraint>(_BSplineVel, ctl.robots(), ctl.robot().robotIndex(), ctl.robot().frame(ctl.hammer_head_frame_name), ctl.nail_normal_vector_world_frame, ctl._lambda_high, ctl._lambda_low, ctl._delta_t, ctl._c_res, ctl._dt_multi, ctl.logger(), ctl._tau_high_mulitplier, ctl._K, &ctl._Activation_height);
    mc_rtc::log::info("linear impulsive torque constraint activated");
  }
  else{
    ctl.impulseConstraint = std::make_unique<mc_solver::ImpulseConstraint>(ctl.robots(), ctl.robot().robotIndex(), ctl.robot().frame(ctl.hammer_head_frame_name), ctl.nail_normal_vector_world_frame, ctl._lambda_high, ctl._lambda_low, ctl._delta_t, ctl._c_res, ctl._dt_multi, ctl.logger());
    mc_rtc::log::info("impulsive torque constraint activated");
  }

  dimweights_transform_task = {0,0,0,0,0,0};
  previous_effective_mass = ctl.effective_mass;
}


bool Get_In_Position_Task::run(mc_control::fsm::Controller & ctl_)
{
//   // Stagger the buildup of the task stiffness for the transform task to not get a failing QP on startup of state
//   double error = gripper_task->eval().norm();
//   double k_p = _gripper_task_min_stiffness;
//   if(first_iteration){
//     first_instance_error = error;
//     first_iteration = false;
//   }
// // This function uses a scaled and shifted Hyperbolic Tangent (tanh) function to provide a
// // non-linear, smooth, and bounded stiffness gain. The stiffness increases smoothly as the
// // error approaches the goal, preventing abrupt, unstable jumps in control effort near
// // the target while ensuring the stiffness is capped at K_max.
// //
// // The core relationship is designed such that:
// // - As error -> first_instance_error (large), k_p -> min_stiffness (K_min).
// // - As error -> goal_error (small), k_p -> max_stiffness (K_max).
//   k_p = _gripper_task_min_stiffness + (_gripper_task_max_stiffness-_gripper_task_min_stiffness) * (
//     (tanh((_gripper_task_goal_error - error)/_gripper_task_K_scaling_factor) - tanh((_gripper_task_goal_error - first_instance_error)/_gripper_task_K_scaling_factor)) /
//     (1 - tanh((_gripper_task_goal_error - first_instance_error)/_gripper_task_K_scaling_factor)));
//
//   //  mc_rtc::log::info("Current error is {} thus the stiffness is {}", error, k_p);
//   gripper_task->stiffness(k_p);

  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);
  _new_mbc = ctl.robot().mbc();

  // ------------------------- Logging values ----------------------------

  if (ctl.bspline_active_)
  {
    ctl.hammer_tip_reference_velocity_vector = bezier_vel_from_task(_BSplineVel,
                                                                  ctl);
  } else
  {
    ctl.hammer_tip_reference_velocity_vector = _transform_task->refVelB().linear();
  }

  ndcurves::bezier_curve bezier_curve = *_BSplineVel->spline().get_bezier();
  if (ctl.bspline_active_)
  {
    ctl.hammer_tip_reference_position_vector = bezier_curve(_total_time_elapsed);
  } else
  {
    ctl.hammer_tip_reference_position_vector = gripper_task->target().translation();
  }
  ctl.projected_momentum_of_hammer_tip = compute_projected_momentum(ctl.effective_mass,
                                                                    ctl.hammer_tip_actual_velocity_vector, 
                                                                    ctl.nail_normal_vector_world_frame);
  ctl.bspline_tracking_error = ctl.hammer_tip_actual_position_vector - ctl.hammer_tip_reference_position_vector;
  if (ctl.bspline_active_)
  {
    ctl.bspline_eval = _BSplineVel->evalTracking();
    ctl.bspline_eval_norm = _BSplineVel->evalTracking().norm();
  } else
  {
    ctl.bspline_eval = Eigen::Vector6d::Zero();
    ctl.bspline_eval_norm = 0;
  }
  
    // ------------------------- Impuslvie cosntraint activation ----------------------------


  if (ctl.hammer_tip_reference_velocity_vector.dot(ctl.nail_normal_vector_world_frame)<=0 && !ctl.impulsive_constraint_flag) // activation of the impulseconstraint after distance 
  {
      ctl.impulsive_constraint_flag=true;
      if(!ctl._Activation_height) ctl._Activation_height=(ctl.robot().frame("Hammer_head").position().translation() - _BSplineVel->target().translation()).norm(); 
      assert(ctl._tau_high_mulitplier>=1 && "_tau_high_mulitplier must be at least 1");
      ctl.solver().addConstraintSet(ctl.impulseConstraint);
  }

  
  Eigen::Matrix3d current_hammer_rotation = ctl.robot().frame(ctl.hammer_head_frame_name).position().rotation();
  ctl.vector_orientation_error = vector_error(-ctl.nail_normal_vector_world_frame, 
                                            (current_hammer_rotation.transpose()*ctl.normal_vector_to_align_in_hammerhead_frame).normalized());

  // ------------------------- Effective mass maximization task update ----------------------------
  // See Jimmy paper to understand why we use posture task

  //_gradient_of_m = compute_emass_gradient_three_point_backward_difference_mbc(_new_mbc,
  //                                                                          ctl, 
  //                                                              ctl.nail_normal_vector_world_frame);

  int number_of_joints = ctl.robot().tvmRobot().qJoints()->size();

  Eigen::MatrixXd joint_selector = Eigen::MatrixXd::Zero(number_of_joints, number_of_joints);
  for (const auto joint: mass_maximization_active_joints)
  {
    auto joint_index = jointIndex(joint);
    joint_selector(joint_index, joint_index) = 1.0;
  }

  //Eigen::VectorXd feedforward_term = (ctl._magic_effective_mass_maximization_task_weight/ctl._magic_posture_task_weight) * joint_selector * _gradient_of_m.tail(ctl.robot().tvmRobot().qJoints()->size());
  //ctl.getPostureTask(ctl.robot().name())->refAccel(feedforward_term);

  // ------------------------- Stop hammering conditions  ----------------------------

  ctl.impact_detected = ctl.robot().forceSensor("LeftHandForceSensor").force().norm()>=ctl.magic_force_threshold_sensor;

  double impact_detection_position_threshold = 0.02;
  bool height_stop_offset=0.01;
  bool stop_height_flag = ctl.hammer_tip_actual_position_vector[2] < ctl._magic_hitting_target[2];

  if (!ctl.bspline_active_)
  {
    mc_rtc::log::info("Position {} with goal {}", ctl.hammer_tip_actual_position_vector.transpose(), _end_point.transpose());
  }

  //log bspline point
  if(_total_time_elapsed > (ctl._magic_BSpline_max_duration/*+1.f*/) && stop_height_flag){
    ctl.comparisonRobots_->robot().posW(sva::PTransformd(ctl.floatingBaseSensor_.orientation(), ctl.floatingBaseSensor_.position()));
    ctl.comparisonRobots_->robot().mbc().q = ctl.realRobot().mbc().q;

    Eigen::Vector3d hammer_normal_world_frame = (ctl.robot().frame(ctl.hammer_head_frame_name).position().rotation().transpose()*Eigen::Vector3d(1, 0, 0)).normalized();
    Eigen::Vector3d hammer_normal_world_frame_bodysensor = (ctl.comparisonRobots_->robot().frame(ctl.hammer_head_frame_name).position().rotation().transpose()*Eigen::Vector3d(1, 0, 0)).normalized();

    mc_rtc::log::info("Lowest hammer pos reached");

    mc_rtc::log::info("actual hammer normal in world frame = {}", hammer_normal_world_frame);
    mc_rtc::log::info("target hammer normal in world frame = {}", -ctl.nail_normal_vector_world_frame);
    mc_rtc::log::info("angle error = {} deg", (180/M_PI) * vector_error(hammer_normal_world_frame, -ctl.nail_normal_vector_world_frame));

    ctl.last_hitting_angle = (180/M_PI) * vector_error(hammer_normal_world_frame, -ctl.nail_normal_vector_world_frame);
    ctl.last_hitting_angle_bodysensor = (180/M_PI) * vector_error(hammer_normal_world_frame_bodysensor, -ctl.nail_normal_vector_world_frame);
    ctl.last_hitting_point = ctl.robot().frame(ctl.hammer_head_frame_name).position().translation();
    ctl.last_hitting_point_error_tilt = (ctl.robot().frame(ctl.hammer_head_frame_name).position().translation() - _nail_point).cwiseAbs();
    ctl.last_hitting_point_error_bodysensor = (ctl.comparisonRobots_->robot().frame(ctl.hammer_head_frame_name).position().translation() - _nail_point).cwiseAbs();

    output("STOP");
    return true;
  }

  if(ctl.impact_detected/*stop*/)
  {
    ctl.number_of_hits++;

    // Manually set the floating base pose, velocity and acceleration in the world frame from the bodysensor
    ctl.comparisonRobots_->robot().posW(sva::PTransformd(ctl.floatingBaseSensor_.orientation(), ctl.floatingBaseSensor_.position()));
    ctl.comparisonRobots_->robot().mbc().q = ctl.realRobot().mbc().q;

    Eigen::Vector3d hammer_normal_world_frame = (ctl.robot().frame(ctl.hammer_head_frame_name).position().rotation().transpose()*Eigen::Vector3d(1, 0, 0)).normalized();
    Eigen::Vector3d hammer_normal_world_frame_bodysensor = (ctl.comparisonRobots_->robot().frame(ctl.hammer_head_frame_name).position().rotation().transpose()*Eigen::Vector3d(1, 0, 0)).normalized();

    mc_rtc::log::info("IMPACT DETECTED ON THE NAIL");
    mc_rtc::log::info("This is hit number {}", ctl.number_of_hits);

    mc_rtc::log::info("actual hammer normal in world frame = {}", hammer_normal_world_frame);
    mc_rtc::log::info("target hammer normal in world frame = {}", -ctl.nail_normal_vector_world_frame);
    mc_rtc::log::info("angle error = {} deg", (180/M_PI) * vector_error(hammer_normal_world_frame, -ctl.nail_normal_vector_world_frame));

    mc_rtc::log::info("Impact velocity is: {} and the projected momentum is: {}", ctl.hammer_tip_actual_velocity_vector.transpose(), ctl.projected_momentum_of_hammer_tip);

    ctl.last_hitting_angle = (180/M_PI) * vector_error(hammer_normal_world_frame, -ctl.nail_normal_vector_world_frame);
    ctl.last_hitting_angle_bodysensor = (180/M_PI) * vector_error(hammer_normal_world_frame_bodysensor, -ctl.nail_normal_vector_world_frame);
    ctl.last_hitting_point = ctl.robot().frame(ctl.hammer_head_frame_name).position().translation();
    ctl.last_hitting_point_error_tilt = (ctl.robot().frame(ctl.hammer_head_frame_name).position().translation() - _nail_point).cwiseAbs();
    ctl.last_hitting_point_error_bodysensor = (ctl.comparisonRobots_->robot().frame(ctl.hammer_head_frame_name).position().translation() - _nail_point).cwiseAbs();
    ctl.last_projected_momentum_of_hammer_tip = ctl.projected_momentum_of_hammer_tip;

    ctl.hitting_data_to_log = true;
    ctl.hittingforce_data_to_log = true;

    output("STOP");
    return true;
  } else
  {
    Eigen::Vector3d hammer_normal_world_frame = (ctl.robot().frame(ctl.hammer_head_frame_name).position().rotation().transpose()*Eigen::Vector3d(1, 0, 0)).normalized();
    Eigen::Vector3d hammer_normal_world_frame_bodysensor = (ctl.comparisonRobots_->robot().frame(ctl.hammer_head_frame_name).position().rotation().transpose()*Eigen::Vector3d(1, 0, 0)).normalized();

    ctl.previous_hitting_angle = (180/M_PI) * vector_error(hammer_normal_world_frame, -ctl.nail_normal_vector_world_frame);
    ctl.previous_projected_momentum_of_hammer_tip = ctl.projected_momentum_of_hammer_tip;
    ctl.previous_hitting_angle_bodysensor = (180/M_PI) * vector_error(hammer_normal_world_frame_bodysensor, -ctl.nail_normal_vector_world_frame);
    ctl.previous_hitting_point = ctl.robot().frame(ctl.hammer_head_frame_name).position().translation();
    ctl.previous_hitting_point_error_tilt = (ctl.robot().frame(ctl.hammer_head_frame_name).position().translation() - _nail_point).cwiseAbs();
    ctl.previous_hitting_point_error_bodysensor = (ctl.comparisonRobots_->robot().frame(ctl.hammer_head_frame_name).position().translation() - _nail_point).cwiseAbs();
  }


  if(stop){
    mc_rtc::log::info("Stop button clicked");
    output("STOP");
    return true;
  }
  _total_time_elapsed+=ctl.solver().dt();
  return false;
}

void Get_In_Position_Task::teardown(mc_control::fsm::Controller & ctl_)
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);
  ctl.gui()->removeElement({}, ctl.stop_hammering_button_name);
  if (_BSplineVel)
  {
    ctl.solver().removeTask(_BSplineVel);
    _BSplineVel.reset();
  }
  if (_transform_task)
  {
    ctl.solver().removeTask(_transform_task);
    _transform_task.reset();
  }
  ctl.bspline_active_ = false;

  ctl.trajectories_executed++;

  if (_vectorOrientationTask)
  {
    ctl.solver().removeTask(_vectorOrientationTask);
    _vectorOrientationTask.reset();
  }
  if (ctl.impulseConstraint)
  {
    ctl.solver().removeConstraintSet(*ctl.impulseConstraint);
    ctl.impulseConstraint.reset();
  }
  ctl.impulsive_constraint_flag = false;
  ctl.getPostureTask(ctl.robot().name())->refAccel(Eigen::VectorXd::Zero(35));
  rm_logs(ctl_);
  mc_rtc::log::info("Tasks cleared successfully");
}

const Eigen::Vector3d Get_In_Position_Task::bezier_vel_from_task(
  const std::shared_ptr<mc_tasks::BSplineTrajectoryTask> &BSplineVel,
  mc_control::fsm::Controller & ctl_) const 
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);
  ndcurves::bezier_curve bezier_curve = *BSplineVel->spline().get_bezier();
  Eigen::Vector3d p_past = bezier_curve(_total_time_elapsed - ctl.solver().dt());
  Eigen::Vector3d p_future = bezier_curve(_total_time_elapsed + ctl.solver().dt());

  return (p_future - p_past)/(2*ctl.solver().dt());
}                                                                  

const double Get_In_Position_Task::vector_error(const Eigen::Vector3d &va, const Eigen::Vector3d &vb) const
{
  return std::acos(va.dot(vb)/(va.norm()*vb.norm()));
}

const double Get_In_Position_Task::compute_projected_momentum(
  const double &effective_mass,
  const Eigen::Vector3d &velocity_vector, 
  const Eigen::Vector3d &normal_vector) const
{
  return effective_mass * (velocity_vector.x()*normal_vector.x()
                          + velocity_vector.y()*normal_vector.y()
                          + velocity_vector.z()*normal_vector.z());
}  // TODO: should this not use the 2-norm?

const double Get_In_Position_Task::compute_effective_mass_with_mbc(
  rbd::MultiBodyConfig mbc, 
  mc_control::fsm::Controller & ctl_, 
  const Eigen::Vector3d &normal_vector) const{

  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);
    
  // If you dont put this line the gradient is 0 everywhere because M and J are not updating
  ctl.robot().forwardKinematics(mbc);                                               
                                                        

  rbd::MultiBody robot_mb = ctl_.robot().mb();
  rbd::Jacobian jac(robot_mb, ctl.hammer_head_frame_name);
  Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot_mb, mbc);

  Eigen::MatrixXd full_world_frame_jacobian(6, ctl.robot().mb().nrDof());
  jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);

  rbd::ForwardDynamics fd(robot_mb);
  fd.computeH(robot_mb, mbc);
  Eigen::MatrixXd M = fd.H();

  const Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

  const Eigen::Matrix3d LAMBDA = linear_jacobian*M.inverse()*linear_jacobian.transpose();

  return 1/(normal_vector.transpose()*LAMBDA*normal_vector);
}

const rbd::MultiBodyConfig Get_In_Position_Task::encoders_values_to_mbc(
  const std::vector<double> &encoderValues,
  mc_control::fsm::Controller & ctl_) const
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);
  rbd::MultiBodyConfig mbc_res = ctl.robot().mbc();
  std::vector<double> q_encoders = encoderValues;
  std::map<std::string, std::vector<double>> q_mbc_map;
  std::vector<std::string> ref_joint_order = ctl.robot().refJointOrder();
  std::map<std::string, double> q_encoders_map;
  std::map<std::string, std::vector<double>> res;
  
      
  //Convert the encoderValues vector into some mbc.q vector
  for(size_t i = 0; i < std::size(q_encoders); ++i)
  {
    q_encoders_map[ref_joint_order.at(i)] = q_encoders.at(i);
  }

  for(size_t i = 0; i < std::size(mbc_res.q); ++i)
  {
    q_mbc_map[ctl_.robot().mb().joint(i).name()] = mbc_res.q.at(i);
  }

  for(size_t i = 0; i < std::size(mbc_res.q); ++i)
  {
    std::string jointName = ctl_.robot().mb().joint(i).name();
    try{
      if (std::size(q_mbc_map.at(jointName)) != 1)
      {
        res[jointName] = q_mbc_map.at(jointName);
      }
      else
      {
        res[jointName] = {q_encoders_map.at(jointName)};
      }
    }
    catch (std::out_of_range){
      res[jointName] = q_mbc_map.at(jointName);
    }
    mbc_res.q[i] = res.at(jointName);
  }
  return mbc_res;
}

const double Get_In_Position_Task::compute_effective_mass_with_encoders(
  const std::vector<double> &encoderValues,
  mc_control::fsm::Controller & ctl_, 
  const Eigen::Vector3d &normal_vector) const{

  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);
  rbd::MultiBodyConfig mbc = encoders_values_to_mbc(encoderValues, ctl);
  ctl_.robot().forwardKinematics(mbc);


  rbd::MultiBody robot_mb = ctl_.robot().mb();
  rbd::Jacobian jac(robot_mb, ctl.hammer_head_frame_name);
  Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot_mb, mbc);

  Eigen::MatrixXd full_world_frame_jacobian(6, ctl.robot().mb().nrDof());
  jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);

  Eigen::MatrixXd M = ctl.dynamicsConstraint->motionConstr().fd().H();
  Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

  Eigen::MatrixXd M_inverse = M.inverse();
  Eigen::Matrix3d LAMBDA = linear_jacobian*M_inverse*linear_jacobian.transpose();
  return 1/(normal_vector.transpose()*LAMBDA*normal_vector);
}

const Eigen::VectorXd Get_In_Position_Task::compute_emass_gradient_backward_difference_mbc(
  const rbd::MultiBodyConfig &mbc, 
  mc_control::fsm::Controller &ctl_, 
  const Eigen::Vector3d &normal_vector) const{

  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);
  const double epsilon = 1E-6;
  Eigen::VectorXd grad(ctl.robot().mb().nrDof(), 1);
  grad.setOnes();
  double m_q = compute_effective_mass_with_mbc(mbc, ctl_, normal_vector);
  double backward_effective_mass = 0;
  
  unsigned int j = 0;
  rbd::MultiBodyConfig backward_mbc;
  for(size_t i = 0; i <  std::size(mbc.q); ++i)
  { 
    if(mbc.q.at(i).size() != 1)
    {
      // Ignore floating base and fixed joints
      continue;
    }
    // dqi = mbc.q.at(i).at(0) - _old_mbc.q.at(i).at(0);
    backward_mbc = mbc;
    backward_mbc.q.at(i).at(0) -= epsilon;

    //Finite differences
    backward_effective_mass = compute_effective_mass_with_mbc(backward_mbc, ctl_, normal_vector);
    grad(j,0) = (m_q - backward_effective_mass)/epsilon;
    j+=1;
      
      // mc_rtc::log::info("forward_mbc_q[i] = {}", forward_mbc.q.at(i).at(0));
      // mc_rtc::log::info("backward_mbc_q[i] = {}", backward_mbc.q.at(i).at(0));
      // mc_rtc::log::info("---------------------");
      // mc_rtc::log::info("grad[{}] = {}", 6+j, grad_res);
      // backward_mbc.q.at(i).at(0) = mbc.q.at(i).at(0);
  }
  return grad;
}

const Eigen::VectorXd Get_In_Position_Task::compute_emass_gradient_three_point_backward_difference_mbc(
  const rbd::MultiBodyConfig &mbc, 
  mc_control::fsm::Controller &ctl_, 
  const Eigen::Vector3d &normal_vector) const{

  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);

  const double epsilon = 1E-6;
  Eigen::VectorXd grad(ctl.robot().mb().nrDof(), 1);
  grad.setZero();
  double backward_effective_mass = 0;
  double m_q = compute_effective_mass_with_mbc(mbc, ctl_, normal_vector);

  unsigned int j = 0;
  if (ctl.robot().mb().nrDof() > ctl.robot().tvmRobot().qJoints()->size())
  {
    j = ctl.robot().mb().nrDof() - ctl.robot().tvmRobot().qJoints()->size(); // Start after the floating base DOFs
  }
  rbd::MultiBodyConfig p1;
  rbd::MultiBodyConfig p2;

  rbd::MultiBodyConfig p3;
  rbd::MultiBodyConfig p4;

  double first_term = 0;
  double second_term = 0;


  for(size_t i = 0; i <  std::size(mbc.q); ++i)
  { 
    if(mbc.q.at(i).size() != 1)
    {
      // Ignore floating base and fixed joints
      // TODO: Implement the gradient for the floating base DOFs as well (take in mind to not predefine j to be the size of the floating base DOFs but to update it in the loop)
      continue;
    }
    // dqi = mbc.q.at(i).at(0) - _old_mbc.q.at(i).at(0);
    p1 = mbc;
    p2 = mbc;
    p3 = mbc;
    p4 = mbc;

    p1.q.at(i).at(0) -= 2*epsilon;
    p2.q.at(i).at(0) -= epsilon;

    first_term = compute_effective_mass_with_mbc(p1, ctl_, normal_vector);
    second_term = compute_effective_mass_with_mbc(p2, ctl_, normal_vector);

    //Finite differences
    backward_effective_mass = first_term - 4*second_term + 3*m_q;
    grad(j,0) = backward_effective_mass/(2*epsilon);
    j+=1;
      
      // mc_rtc::log::info("forward_mbc_q[i] = {}", forward_mbc.q.at(i).at(0));
      // mc_rtc::log::info("backward_mbc_q[i] = {}", backward_mbc.q.at(i).at(0));
      // mc_rtc::log::info("---------------------");
      // mc_rtc::log::info("grad[{}] = {}", 6+j, grad_res);
      // backward_mbc.q.at(i).at(0) = mbc.q.at(i).at(0);
  }
  return grad;
}

const Eigen::VectorXd Get_In_Position_Task::compute_emass_gradient_central_difference_mbc(
                                                            const rbd::MultiBodyConfig &mbc, 
                                                             mc_control::fsm::Controller &ctl_, 
                                                            const Eigen::Vector3d &normal_vector) const{
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);

    double dqi = 0;
    Eigen::VectorXd grad(ctl.robot().mb().nrDof(), 1);
    grad.setOnes();

    rbd::MultiBodyConfig forward_mbc;
    rbd::MultiBodyConfig backward_mbc;

    double forward_effective_mass = 0;
    double backward_effective_mass = 0;
    unsigned int j = 0;
    // ctl.robot().forwardKinematics(mbc);
    for(size_t i = 0; i <  std::size(mbc.q); ++i)
    { 
      if(mbc.q.at(i).size() != 1)
      {
        // Ignore floating base (size = 6) and fixed joints (size = 0)
        continue;
      }
      dqi = mbc.q.at(i).at(0) - _old_mbc.q.at(i).at(0);
      forward_mbc = mbc;
      backward_mbc = mbc;
      forward_mbc.q.at(i).at(0) += dqi;
      backward_mbc.q.at(i).at(0) -= dqi;

      //Finite differences - Central differences
      forward_effective_mass = compute_effective_mass_with_mbc(forward_mbc, 
                                                                ctl, 
                                                                  normal_vector);
      backward_effective_mass = compute_effective_mass_with_mbc(backward_mbc, 
                                                                  ctl, 
                                                                  normal_vector);

      grad(j,0) = (forward_effective_mass - backward_effective_mass)/(2*dqi);
      j+=1;
    }
  
  return grad;
}

const double Get_In_Position_Task::estimate_previous_effective_mass(const Eigen::VectorXd &gradient, const double &current_m) const{
  double dm_sum = 0;
  double dqi = 0;
  std::vector<std::vector<double>> q = _new_mbc.q;
  std::vector<std::vector<double>> old_q = _old_mbc.q;
  unsigned int j = 0;
  for (size_t i = 0; i < std::size(q); ++i)
  {
    if (std::size(q.at(i)) != 1)
    {
      //Ignore floating base (size = 6) and empty dofs (size = 0)
      continue;
    }    
    dqi = q.at(i).at(0) - old_q.at(i).at(0);
    dm_sum += gradient(j, 0) * dqi;
    j+=1;
  }
  return current_m - dm_sum;
}

void Get_In_Position_Task::load_params()
{
  // Paramters changeable using GUI have been moove to HammeringTaskNew cpp

  std::string magic_values_key = "magic_values";

  _logging_freq = _config(magic_values_key)("logging_freq");

  _gripper_task_weight = _config(magic_values_key)("gripper_task_weight");
  _gripper_task_min_stiffness = _config(magic_values_key)("gripper_task_min_stiffness");
  _gripper_task_max_stiffness = _config(magic_values_key)("gripper_task_max_stiffness");
  _gripper_task_goal_error = _config(magic_values_key)("gripper_task_goal_error");
  _gripper_task_K_scaling_factor = _config(magic_values_key)("gripper_task_s");

  // ------------------------ Loading init and start velocities, accelerations and jerks ---------------------------

  std::string curve_constraints_key = "curve_constraints";
  std::string linear_velocity_key = "linear_velocity";
  std::string linear_acceleration_key = "linear_acceleration";
  std::string x_key = "x";
  std::string y_key = "y";
  std::string z_key = "z";

  std::string init_key = "init";
  std::string end_key = "end";

  _constr.init_acc.x() = _config(curve_constraints_key)(linear_acceleration_key)(x_key)(init_key);
  _constr.init_acc.y() = _config(curve_constraints_key)(linear_acceleration_key)(y_key)(init_key);
  _constr.init_acc.z() = _config(curve_constraints_key)(linear_acceleration_key)(z_key)(init_key);

  _constr.end_acc.x() = _config(curve_constraints_key)(linear_acceleration_key)(x_key)(end_key);
  _constr.end_acc.y() = _config(curve_constraints_key)(linear_acceleration_key)(y_key)(end_key);
  _constr.end_acc.z() = _config(curve_constraints_key)(linear_acceleration_key)(z_key)(end_key);
}

void Get_In_Position_Task::add_logs(mc_control::fsm::Controller & ctl_)
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);

  ctl.logger().addLogEntry("GetInPoseTask_Hammer tip reference bezier velocity [m/s]", this, [&, this]()
  {return ctl.hammer_tip_reference_velocity_vector;});

  ctl.logger().addLogEntry("GetInPoseTask_Hammer tip reference bezier position [m]", this, [&, this]()
  {return ctl.hammer_tip_reference_position_vector;});

  ctl.logger().addLogEntry("GetInPoseTask_Bspline tracking error [m]", this, [&, this]()
  {return ctl.bspline_tracking_error;});

  ctl.logger().addLogEntry("GetInPoseTask_Vector orientation error", this, [&, this]()
  {return ctl.vector_orientation_error*180/M_PI;});

  ctl.logger().addLogEntry("GetInPoseTask_Bspline eval tracking", this, [&, this]()
  {return ctl.bspline_eval;});

  ctl.logger().addLogEntry("GetInPoseTask_Bspline eval", this, [&, this]()
  {return _BSplineVel->eval().norm();});

  ctl.logger().addLogEntry("GetInPoseTask_Bspline eval norm", this, [&, this]()
  {return ctl.bspline_eval_norm;});

  ctl.logger().addLogEntry("impact_detected",this,[&,this]()
  {return ctl.impact_detected*100;});

  ctl.logger().addLogEntry("impulsive constraint activation flag",this,[&,this]()
  {return ctl.impulsive_constraint_flag*100;});

}

void Get_In_Position_Task::rm_logs(mc_control::fsm::Controller & ctl_)
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);

  ctl.logger().removeLogEntry("GetInPoseTask_Hammer tip reference bezier velocity [m/s]");
  ctl.logger().removeLogEntry("GetInPoseTask_Hammer tip reference bezier position [m]");
  ctl.logger().removeLogEntry("GetInPoseTask_Bspline tracking error [m]");
  ctl.logger().removeLogEntry("GetInPoseTask_Vector orientation error");
  ctl.logger().removeLogEntry("GetInPoseTask_Bspline eval tracking");
  ctl.logger().removeLogEntry("GetInPoseTask_Bspline eval");
  ctl.logger().removeLogEntry("GetInPoseTask_Bspline eval norm");
  ctl.logger().removeLogEntry("GetInPoseTask_angle eval");
  ctl.logger().removeLogEntry("impact_detected");
  ctl.logger().removeLogEntry("impulsive constraint activation flag");

}

EXPORT_SINGLE_STATE("Get_In_Position_Task", Get_In_Position_Task)