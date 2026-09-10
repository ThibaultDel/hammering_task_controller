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
  load_params();
  add_logs(ctl_);
 
  // Add a stop button to the gui
  ctl.gui()->addElement({}, mc_rtc::gui::Button(ctl.stop_hammering_button_name, [this]() { stop = true; }));

  // ------------------------- BSplineTrajectoryTask ----------------------------
  
  
  // I dont need to specify the endpoint as a posWp because _target takes care of that
  // If I do specify it, then it would add 1 degree to the curve even though the points are the same
  // I also dont need to specify the starting point because the first argument of the task takes care of that
  // Thus _posWp is empty
  // _posWp = {};
  
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
  _nail_point = _magic_hitting_target;
  _end_point = _nail_point + Eigen::Vector3d(0, 0, 0);
  _posWp = {/*_nail_point*/};

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

  //Eigen::Vector6d dimweights = _BSplineVel->dimWeight(); // dimweigth to be deleted
  // Remove the orientation part of the BSpline by setting the orientation weights to 0
  // if(!_enable_BSpline_orientation)
  // {
  //   dimweights(0) = 0;
  //   dimweights(1) = 0;
  //   dimweights(2) = 0;
  // }
  // Increase the weights on the x and y coordinates
  // dimweights(0) = _magic_BSpline_task_dimweight_rx; // dimweigth to be deleted
  // dimweights(1) = _magic_BSpline_task_dimweight_ry;
  // dimweights(2) = _magic_BSpline_task_dimweight_rz;
  // dimweights(3) = _magic_BSpline_task_dimweight_tx;
  // dimweights(4) = _magic_BSpline_task_dimweight_ty;
  // dimweights(5) = _magic_BSpline_task_dimweight_tz;
  _BSplineVel->dimWeight(ctl.dimweights);
  ctl.solver().addTask(_BSplineVel);
  ctl.bspline_active_ = true;

    // _constr.init_vel = ctl.hammer_tip_actual_velocity_vector;
    // _constr.end_vel = ctl.nail_rot.transpose()*_magic_normal_final_velocity;
    //
    // // The target is the translation of the nail
    // Eigen::Vector3d _nail_point = ctl.robots().robot(ctl.nail_robot_name).frame(ctl.nail_frame_name).position().translation();
    // _end_point = _nail_point + Eigen::Vector3d(0, 0, -0.1);
    //
    // _target = sva::PTransformd(sva::RotX(M_PI)) * sva::PTransformd(sva::RotY(M_PI/2)) * sva::PTransformd(ctl.robots().robot(ctl.nail_robot_name).frame(ctl.nail_frame_name).position().rotation()) *sva::PTransformd(_end_point);//* sva::PTransformd(Eigen::Vector3d(0.5, 0.2, 1));
    //
    // mc_rtc::log::info("Adding BSpline task with weight {}", _magic_BSpline_task_weight);
    //
    // _BSplineVel2 = std::make_shared<mc_tasks::ExactCubicTrajectoryTask>(ctl.robot().frame(ctl.hammer_head_frame_name),
    //                                                                 _magic_normal_final_velocity, // duration depends on the final velocity along z
    //                                                                 _magic_BSpline_task_stiffness,
    //                                                                 _magic_BSpline_task_weight,
    //                                                                 _target,
    //                                                                 _posWp,
    //                                                                 ctl.nail_rot.transpose()*_magic_normal_final_velocity,
    //                                                                 Eigen::Vector3d(0, 0, 0),
    //                                                                 Eigen::Vector3d(0, 0, 0),
    //                                                                 Eigen::Vector3d(0, 0, 0),
    //                                                                 Eigen::Vector3d(0, 0, 0),
    //                                                                 _oriWp);
    //
    // Eigen::Vector6d dimweights = _BSplineVel2->dimWeight();
    // // Remove the orientation part of the BSpline by setting the orientation weights to 0
    // if(!_enable_BSpline_orientation)
    // {
    //   dimweights(0) = 0;
    //   dimweights(1) = 0;
    //   dimweights(2) = 0;
    // }
    // // Increase the weights on the x and y coordinates
    // dimweights(3) = _magic_BSpline_task_dimweight_x;
    // dimweights(4) = _magic_BSpline_task_dimweight_y;
    // dimweights(5) = _magic_BSpline_task_dimweight_z;
    // _BSplineVel2->dimWeight(dimweights);
    // ctl.solver().addTask(_BSplineVel2);
    //


  mc_rtc::log::info("Degree of BSpline : {}", _BSplineVel->spline().get_bezier()->degree());

  ctl.getPostureTask(ctl.robot().name())->stiffness(_magic_posture_task_stiffness);
  ctl.getPostureTask(ctl.robot().name())->weight(ctl._magic_posture_task_weight);

  // gripper_task->target(sva::PTransformd(sva::RotY(M_PI)) * sva::PTransformd(sva::RotZ(M_PI/2)) * sva::PTransformd(Eigen::Vector3d(0 ,0, 0.025)) * ctl.robot("box").frame("Right").position());

  auto gripper_target = sva::PTransformd(Eigen::Quaterniond(0.0f, 0.708, 0.0f, -0.705)) * sva::PTransformd(_end_point);//sva::PTransformd(Eigen::Vector3d(0.7, 0.5, 1)) * 

  // Test transform task
  gripper_task = std::make_shared<mc_tasks::TransformTask>(ctl.robot().frame(ctl.hammer_head_frame_name), _gripper_task_min_stiffness, _gripper_task_weight);
  // ctl.solver().addTask(gripper_task);
  gripper_task->target(gripper_target);

  Eigen::Vector6d dimweights_grip = gripper_task->dimWeight();
  // Remove the orientation part of the BSpline by setting the orientation weights to 0
  // if(!_enable_BSpline_orientation)
  // {
  //   dimweights_grip(0) = 0;
  //   dimweights_grip(1) = 0;
  //   dimweights_grip(2) = 0;
  // }
  // Increase the weights on the x and y coordinates
  // dimweights_grip(0) = _magic_BSpline_task_dimweight_rx;
  // dimweights_grip(1) = _magic_BSpline_task_dimweight_ry;
  // dimweights_grip(2) = _magic_BSpline_task_dimweight_rz;
  // dimweights_grip(3) = _magic_BSpline_task_dimweight_tx;
  // dimweights_grip(4) = _magic_BSpline_task_dimweight_ty;
  // dimweights_grip(5) = _magic_BSpline_task_dimweight_tz;
  // gripper_task->dimWeight(dimweights_grip);


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

  // // Add impulse constraint
  // Eigen::Vector3d normal_nail = ctl.robot(ctl.nail_robot_name).frame(ctl.nail_frame_name).position().rotation().col(2).eval();
  if(ctl.linear_impulsive_torque_ctr_flag){
    Eigen::VectorXd tau_high = ctl.robot().tvmRobot().limits().tu*ctl._tau_high_mulitplier;
    ctl.impulseConstraint = std::make_unique<mc_solver::ImpulseConstraint>(_BSplineVel, ctl.robots(), ctl.robot().robotIndex(), ctl.robot().frame(ctl.hammer_head_frame_name), ctl.nail_normal_vector_world_frame, ctl._lambda_high, ctl._lambda_low, ctl._delta_t, ctl._c_res, ctl._dt_multi, ctl.logger(), tau_high, ctl._K, &ctl._Activation_height);
  }
  else{
    ctl.impulseConstraint = std::make_unique<mc_solver::ImpulseConstraint>(ctl.robots(), ctl.robot().robotIndex(), ctl.robot().frame(ctl.hammer_head_frame_name), ctl.nail_normal_vector_world_frame, ctl._lambda_high, ctl._lambda_low, ctl._delta_t, ctl._c_res, ctl._dt_multi, ctl.logger());
  }
  // Post Bspline velocity task
  // _target_velocity = ctl.nail_rot.transpose()*_magic_normal_final_velocity;
  // _target_vel = sva::MotionVecd(Eigen::Vector3d::Zero(), _target_velocity);
  // _transform_task = std::make_shared<mc_tasks::TransformTask>(ctl.robot().frame(ctl.hammer_head_frame_name), _velocity_task_stiffness, _velocity_task_weight);
  // _transform_task->targetVel(_target_vel);
  // _transform_task->setGains(0, _velocity_task_stiffness);
  //
  dimweights_transform_task = {0,0,0,0,0,0};
  // // Remove the orientation part of the BSpline by setting the orientation weights to 0
  // if(!_enable_BSpline_orientation)
  // {
  //   dimweights_transform_task(0) = 0;
  //   dimweights_transform_task(1) = 0;
  //   dimweights_transform_task(2) = 0;
  // }
  // // Increase the weights on the x and y coordinates
  // dimweights_transform_task(3) = _magic_BSpline_task_dimweight_x;
  // dimweights_transform_task(4) = _magic_BSpline_task_dimweight_y;
  // dimweights_transform_task(5) = _magic_BSpline_task_dimweight_z*2000.f;
  // _transform_task->dimWeight(dimweights_transform_task);
  // ctl.solver().addTask(_transform_task);

  _new_mbc = ctl.robot().mbc();

  //ctl.effective_mass = /*ctl.*/compute_effective_mass_with_mbc(_new_mbc, ctl, ctl.nail_normal_vector_world_frame);

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
  // if(_first_iteration)
  // {
  // // For some reason the mass matrix is null at the very first iteration, Thomas said it was a bug getting fixed
  // _first_iteration = !_first_iteration;
  //   _total_time_elapsed+=ctl.solver().dt();
  //   return false;
  // }

  // Compute some values and update the logs
  // ctl.effective_mass = compute_effective_mass_with_mbc(_new_mbc,
  //                                                 ctl, 
  //                                                 ctl.nail_normal_vector_world_frame);

  //ctl.effective_mass = compute_effective_mass_with_mbc(_new_mbc, ctl, ctl.nail_normal_vector_world_frame);
  //ctl.effective_mass_diff = (ctl.effective_mass - previous_effective_mass)/ctl.solver().dt();
  //ctl.effective_mass_diff_diff = (ctl.effective_mass_d - effective_mass_d)/ctl.solver().dt();
  //previous_effective_mass = ctl.effective_mass;
  //previous_eff_mass_diff = ctl.effective_mass_d;

  // ctl.hammer_tip_actual_velocity_vector = ctl.robot().frame(ctl.hammer_head_frame_name).velocity().linear();
  // ctl.hammer_tip_actual_position_vector = ctl.robot().frame(ctl.hammer_head_frame_name).position().translation();
  if (ctl.bspline_active_)
  {
    ctl.hammer_tip_reference_velocity_vector = bezier_vel_from_task(_BSplineVel,
                                                                  ctl);
  } else
  {
    // const auto sva_vel = _transform_task->refVelB();
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
  // ctl.hammer_tip_reference_position_vector = bezier_curve(_total_time_elapsed);
  // ctl.hammer_tip_reference_position_vector = gripper_task->target().translation();
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
  if (ctl.hammer_tip_reference_velocity_vector.dot(ctl.nail_normal_vector_world_frame)<=0 && !ctl.impulsive_constraint_flag) // activation of the impulseconstraint after distance 
  {
      ctl.impulsive_constraint_flag=true;
      if(!ctl._Activation_height) ctl._Activation_height=_BSplineVel->eval().norm(); 
      assert(ctl._tau_high_mulitplier<=1 && "_tau_high_mulitplier must be more than 1");
      ctl.solver().addConstraintSet(ctl.impulseConstraint);
  }

  Eigen::Matrix3d current_hammer_rotation = ctl.robot().frame(ctl.hammer_head_frame_name).position().rotation();
  ctl.vector_orientation_error = vector_error(-ctl.nail_normal_vector_world_frame, 
                                            (current_hammer_rotation.transpose()*ctl.normal_vector_to_align_in_hammerhead_frame).normalized());

  _gradient_of_m = compute_emass_gradient_three_point_backward_difference_mbc(_new_mbc,
                                                                            ctl, 
                                                                ctl.nail_normal_vector_world_frame);

  // "Modified" posture task or trick
  int number_of_joints = ctl.robot().tvmRobot().qJoints()->size();

  Eigen::MatrixXd joint_selector = Eigen::MatrixXd::Zero(number_of_joints, number_of_joints);
  for (const auto joint: mass_maximization_active_joints)
  {
    auto joint_index = jointIndex(joint);
    joint_selector(joint_index, joint_index) = 1.0;
  }

  Eigen::VectorXd feedforward_term = (ctl._magic_effective_mass_maximization_task_weight/ctl._magic_posture_task_weight) * joint_selector * _gradient_of_m.tail(ctl.robot().tvmRobot().qJoints()->size());
  ctl.getPostureTask(ctl.robot().name())->refAccel(feedforward_term);

  auto q_d = ctl.robot().tvmRobot().alpha()->value();
  Eigen::VectorXd q_d_selected = q_d.tail(number_of_joints);

  ctl.eff_mass_diff_checker = _gradient_of_m.transpose()*q_d;

  // End state at impact
  //ctl.impact_detected = abs(ctl.nail_force_vector.x()) >= ctl.magic_force_threshold_nail || 
  //                      abs(ctl.nail_force_vector.y()) >= ctl.magic_force_threshold_nail || 
  //                      abs(ctl.nail_force_vector.z()) >= ctl.magic_force_threshold_nail;

  ctl.impact_detected = ctl.robot().forceSensor("LeftHandForceSensor").force().norm()>=ctl.magic_force_threshold_sensor;

  double impact_detection_position_threshold = 0.02;
  bool height_stop_offset=0.01;
  bool stop_height_flag = ctl.hammer_tip_actual_position_vector[2] < _magic_hitting_target[2];

  if (!ctl.bspline_active_)
  {
    mc_rtc::log::info("Position {} with goal {}", ctl.hammer_tip_actual_position_vector.transpose(), _end_point.transpose());
  }

  //log bspline point

  if(ctl.text_log_flag){
    ctl.text_log_flag=0;
    int bspline_number_of_points=1000;
    _BSplineVel->spline().samplingPoints(bspline_number_of_points);
    std::ofstream outputFile("/home/thibault/bspline.txt");

    if (!outputFile.is_open()) {
        mc_rtc::log::info("Error: Could not create or open the file!");
    }

    for (const auto& vec : _BSplineVel->spline().sampleTrajectory()) {
        outputFile << vec.transpose() << "a";
    }

    // 4. Close the file stream to save changes and free resources
    outputFile.close();
    
    mc_rtc::log::info("b spline text file written");
  }

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

    //I don't know why but apparently TVMImpulsiveConstraint is not recognised as a type
    //ctl.logger().addLogEntry("ImpulsiveTorqueSimulated_Cut", this, [&, this]()
    //{return static_cast<TVMImpulseConstraint *>(ctl.impulseConstraint->getConstraint().get())->impFunctionLow()->ImpulsiveTorqueTrue();});
    //ctl.logger().removeLogEntry("ImpulsiveTorqueSimulated_Cut");

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
  mc_rtc::log::info("test1");
  ctl.gui()->removeElement({}, ctl.stop_hammering_button_name);
  // ctl.solver().removeTask(gripper_task);
  if (ctl.bspline_active_)
  {
    ctl.solver().removeTask(_BSplineVel);
    ctl.bspline_active_ = false;
  } else
  {
    ctl.solver().removeTask(_transform_task);
  }

  ctl.trajectories_executed++;
  mc_rtc::log::info("test1");

  ctl.solver().removeTask(_vectorOrientationTask);
  ctl.solver().removeConstraintSet(ctl.impulseConstraint);
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

// const double Get_In_Position_Task::compute_effective_mass_with_mbc(
//   rbd::MultiBodyConfig mbc, 
//   mc_control::fsm::Controller & ctl_, 
//   const Eigen::Vector3d &normal_vector) const{
//   auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
//   // If you dont put this line the gradient is 0 everywhere because M and J are not updating
//   ctl.robot().forwardKinematics(mbc);                                                                                                   
//   rbd::MultiBody robot_mb = ctl_.robot().mb();
//   rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
//   Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot_mb, mbc);
//   Eigen::MatrixXd full_world_frame_jacobian(6, ctl.robot().mb().nrDof());
//   jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);
//   const Eigen::MatrixXd M = _dynamicsConstraint->motionConstr().fd().H();
//   const Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);
//   // writeEigenMatrixToCSV(linear_jacobian, "linear_jac.csv");
//   // mc_rtc::log::info("Dim of M = {} x {}", M.cols(), M.rows());
//   // mc_rtc::log::info("M = {}", M);
//   // const Eigen::Matrix3d LAMBDA = linear_jacobian*M.inverse()*linear_jacobian.transpose();
//   const Eigen::MatrixXd LAMBDA_INVERSE = (full_world_frame_jacobian*M*full_world_frame_jacobian.transpose()).inverse();
//   const Eigen::MatrixXd LINEAR_LAMBDA_INVERSE = LAMBDA_INVERSE.block(0, 0, 3, 3);
//   // mc_rtc::log::info("Dim of  LINEAR LAMBDA = {} x {}", LINEAR_LAMBDA.cols(), LINEAR_LAMBDA.rows());
//   // return 1/(normal_vector.transpose()*LAMBDA*normal_vector);
//   return 1/(normal_vector.transpose()*LINEAR_LAMBDA_INVERSE*normal_vector);
// }

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

const Eigen::VectorXd Get_In_Position_Task::compute_emass_gradient_central_difference_encoders(
                                                            const std::vector<double> &encoderValues, 
                                                             mc_control::fsm::Controller &ctl_, 
                                                            const Eigen::Vector3d &normal_vector) const{

    // NOT FINISHED !!!!
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
    double dqi = 0; 
    Eigen::VectorXd grad(ctl.robot().mb().nrDof(), 1);
    grad.setOnes();

    std::vector<std::string> unusable_dofs;
    for (size_t i = 0; i < std::size(ctl_.robot().mbc().q); ++i)
    {
      if (std::size(ctl_.robot().mbc().q.at(i)) != 1)
      {
        unusable_dofs.push_back(ctl_.robot().mb().joint(i).name());
      }
    }
    // std::map<std::string, double> encoder_values_map;
    // for (size_t i = 0; i < std::size(encoderValues); ++i)
    // {
    //   encoder_values_map[ctl_.robot().refJointOrder().at(i)] = encoderValues.at(i);
    // }
    // std::vector<double> reorg_encoder_values;
    // for (size_t i = 0; i < std::size(usable_dofs); ++i)
    // {
    //   reorg_encoder_values.at(i) = encoder_values_map.at(usable_dofs.at(i));
    // }    

    double forward_effective_mass = 0;
    double backward_effective_mass = 0;

    for(size_t i = 0; i < 6; ++i)
    {
      //We suppose that the floating base does not move, hence the first 6 DOFs gradient wrt to q are 0
      grad(i,0) = 0;
    }
    unsigned int j = 6; // Skipping the first 6 rows representing the floating base
    for(size_t i = 0; i <  std::size(encoderValues); ++i)
    { 
      // What dof i am touching : if it is an unactuated dof i should skip
      if (std::find(unusable_dofs.begin(), 
                      unusable_dofs.end(), 
                      ctl_.robot().refJointOrder().at(i)) != unusable_dofs.end())
                      //refJointOrder should be the same as the mbc order ?
      {
        continue;
      }
      std::vector<double> forward_encoder_values = encoderValues;
      std::vector<double> backward_encoder_values = encoderValues;
      
      dqi = encoderValues.at(i) - _old_q_encoders.at(i);
      forward_encoder_values.at(i) += dqi;
      backward_encoder_values.at(i) -= dqi;

      //Finite differences - Central differences
      forward_effective_mass = compute_effective_mass_with_encoders(forward_encoder_values, 
                                                                                ctl_, 
                                                                                  normal_vector);
      backward_effective_mass = compute_effective_mass_with_encoders(backward_encoder_values, 
                                                                                ctl_, 
                                                                                  normal_vector);
      grad(j,0) = (forward_effective_mass - backward_effective_mass)/(2*dqi);
      j+=1;
      
    }
  
  return grad;
}
const double Get_In_Position_Task::compute_effective_mass_naive(rbd::MultiBodyConfig mbc, mc_control::fsm::Controller & ctl_) const
{
  // ctl_.robot().forwardKinematics(mbc);
  // auto robot_mb = ctl_.robot().mb();
  // _dynamicsConstraint->motionConstr().fd_non_const().computeH(robot_mb, q_test);
  // double delta_psi = 0.01; // rad

  // std::cout << "nrDOF = " << ctl.robot().mb().nrDof() << std::endl;
  // std::cout << "size = " << std::size(qf.q) << std::endl;
  // int n = 0;
  std::cout << "qf = {"  << std::endl;

  // Store only the actuated dofs in Q
  // std::vector<double> Q = {};
  for (size_t i = 0; i < std::size(mbc.q); ++i)
  {
    std::cout << ctl_.robot().mb().joint(i).name()<< " : " ; 
    std::cout << "{";
    for(size_t j = 0; j < std::size(mbc.q.at(i)); ++j)
    {
      
      std::cout << mbc.q.at(i).at(j) << ", ";
  //     n+=1;
  //     Q.push_back(qf.q.at(i).at(j));
    }
    std::cout << "}" << std::endl;
  }
  std::cout << "}" << std::endl;
  // std::cout << "n = " << n << std::endl;

  // std::cout << "Q = {"  << std::endl;
  // for (size_t i = 0; i < std::size(Q); ++i){
  //   std::cout << Q.at(i) << std::endl;
  // }
  // std::cout << "}"  << std::endl;
  // Eigen::MatrixXd M = _dynamicsConstraint->motionConstr().fd().H();
  //   // std::cout << "M  = \n" << M << "\n";
  // Eigen::MatrixXd M_inverse = M.inverse();
  // // std::cout << "Creating my Jacobian" << std::endl;
  // rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
  // auto world_frame_jacobian = jac.jacobian(robot_mb,mbc);
  // // std::cout << "Created world frame jacobian" << std::endl;

  // Eigen::MatrixXd full_world_frame_jacobian(6, ctl_.robot().mb().nrDof());
  // jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);
  // // std::cout << "Projected my world frame Jacobian" << std::endl;
  
  // Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);
  // std::cout << "Created linear jacobian" << std::endl;

  // Eigen::MatrixXd angular_jacobian = full_world_frame_jacobian.bottomRows(3);
  // std::cout << "Created angular jacobian" << std::endl;

  // std::cout << "dim angular jacobian = " << angular_jacobian.rows() << " x " << angular_jacobian.cols() << std::endl;
  // Eigen::MatrixXd pseudo_inverted_angular_jacobian = angular_jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(angular_jacobian.rows(), angular_jacobian.rows()));
  // std::cout << "Created pseudo inverse" << std::endl;
  
  // auto pseudo_inverted_angular_jacobian_third_column = pseudo_inverted_angular_jacobian.col(3 - 1);
  // std::cout << "Created pseudo inverse angular jac" << std::endl;

  // Eigen::MatrixXd delta_x_omega(3, 1);
  // std::cout << "Created delta x omega" << std::endl;

  // delta_x_omega << 0, 0, delta_psi;

  // Eigen::MatrixXd delta_q =  pseudo_inverted_angular_jacobian * delta_x_omega;
  // std::cout << "Created delta q" << std::endl;e

  // std::cout << "JointConfig = { " << std::endl;
  // for(size_t i = 0; i < std::size(qf.jointConfig); ++i)
  // {
  //   std::cout << qf.jointConfig.at(i) << std::endl;
  // }
  // std::cout << "}" << std::endl;

  // std::cout << "delta q = " << delta_q << std::endl;

  // auto incremented_robot_config = qf.q;
  // incremented_robot_config.at(37);
  
  // std::cout << "Joint " << ctl_.robot().mb().jointIndexByName("LWRY") << std::endl;
  // std::cout << "Joint value at 37 : " << Q.at(37 + 1) << std::endl;

  
  // std::cout << M_inverse << std::endl;
  // std::cout << "Created M inverse" << std::endl;
  // std::cout << "Size M^-1 = " << M_inverse.rows() << " x " << M_inverse.cols() << std::endl;
  // writeEigenMatrixToCSV(M, "MassMatrixMcRTC.csv");
  // writeEigenMatrixToCSV(full_world_frame_jacobian, "full_world_frame_jacobian.csv");
  // std::cout << M << std::endl;
  
  
  
  double inner_sum = 0;
  double J_nu_3_j = 0;
  double J_nu_3_k = 0;
  double M_inverse_j_k = 0;
  
  double outer_sum = 0;
  double effective_mass = 0;
  double delta_psi_tot = 0;
  // auto q_start = qf;
  // std::cout << "Starting double sum" << std::endl;
  
  // while(delta_psi_tot < 2*M_PI)
  // {
    // std::cout << "delta_psi_tot = " << delta_psi_tot << std::endl;
    //Create everything

    // std::cout << M_inverse.row(0) << std::endl;
    // world_frame_jacobian = jac.jacobian(robot_mb,q_test);

    // Eigen::MatrixXd full_world_frame_jacobian(6, robot_mb.nrDof());
    // jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);
    
    // Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

    // Eigen::MatrixXd angular_jacobian = full_world_frame_jacobian.bottomRows(3);

    // std::cout << "dim angular jacobian = " << angular_jacobian.rows() << " x " << angular_jacobian.cols() << std::endl;
    // Eigen::MatrixXd pseudo_inverted_angular_jacobian = angular_jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(angular_jacobian.rows(), angular_jacobian.rows()));
    
    // pseudo_inverted_angular_jacobian_third_column = pseudo_inverted_angular_jacobian.col(3 - 1);

    // Eigen::MatrixXd delta_x_omega(3, 1);

    // Eigen::MatrixXd pseudo_inverted_full_jac = full_world_frame_jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(full_world_frame_jacobian.rows(), full_world_frame_jacobian.rows()));
    // delta_x_omega << 0, 0, delta_psi;
    // Eigen::MatrixXd delta_x(6, 1);
    // delta_x << 0, 0, 0, 0, 0, delta_psi;
    // Eigen::MatrixXd delta_q =  pseudo_inverted_angular_jacobian * delta_x_omega;
    // Eigen::MatrixXd delta_q =  pseudo_inverted_full_jac* delta_x;


    // Compute Outer sum
    // outer_sum = 0;
    // for(int k=0; k <= ctl.robot().mb().nrDof() - 1 ; ++k)
    // {
    //   // Inner sum
    //   inner_sum = 0;
    //   for(int j=0; j <= ctl.robot().mb().nrDof() - 1; ++j)
    //   { 
    //     J_nu_3_j = linear_jacobian(3-1,j);
    //     // std::cout << "Computed linear jac (3-1, "  << j  << ") = " << linear_jacobian(3-1, j) << std::endl;
    //     M_inverse_j_k = M_inverse(j, k);
    //     // std::cout << "M_inverse (" << j << "," << k << ") = " << M_inverse_j_k << std::endl;
    //     inner_sum += J_nu_3_j * M_inverse_j_k;
    //   }
    //   J_nu_3_k = linear_jacobian(3-1, k);
    //   outer_sum += inner_sum*J_nu_3_k;
    // }

    // std::cout << "Testing effective mass : "<< 1/outer_sum << std::endl;
    // if(1/outer_sum > effective_mass)
    // {
    //   effective_mass = 1/outer_sum;
    // }

    // std::cout << "deltaq =  " << delta_q << std::endl;

    // printConfig(q_test, "q_test before = {");        
    // increment q_test by delta q
    // int L = 0;
    // int m = 0;
    // for (size_t i = 0; i < std::size(q_test.q); ++i)
    // {
    //   for(size_t j = 0; j < std::size(q_test.q.at(i)); ++j)
    //   {
    //     q_test.q.at(i).at(j) += delta_q(j+L, 0);
    //     m += 1;
    //   }
    //   L += m;
    //   m = 0;
    // }

    // printConfig(q_test, "q_test after = {");        
  //   delta_psi_tot += delta_psi;
  // }
  return 1/outer_sum;
}

void Get_In_Position_Task::printConfig(rbd::MultiBodyConfig mbc, std::string string) const
{
    std::cout << string << std::endl;
    for (size_t i = 0; i < std::size(mbc.q); ++i)
    {
      for(size_t j = 0; j < std::size(mbc.q.at(i)); ++j)
      {
        std::cout << mbc.q.at(i).at(j) <<"," << std::endl;
      }
    }
    std::cout << "}" << std::endl;
}

void Get_In_Position_Task::writeEigenMatrixToCSV(const Eigen::MatrixXd& matrix, const std::string& filename) const {
    // std::cout << "entering write eigen function" << std::endl;
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file!" << std::endl;
        return;
    }

    // std::cout << "For loop writeEigen begin" << std::endl;
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            file << matrix(i, j);
            if (j < matrix.cols() - 1) file << ",";  // Separate columns by commas
        }
        file << "\n";  // Newline for each row
    }
    file.close();
    // std::cout << "Matrix written to " << filename << std::endl;
}
void Get_In_Position_Task::writeVectorToCSV(const std::vector<double> & vec, const std::string & filename) const 
{
    std::ofstream file(filename);
    if(!file.is_open())
    {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }

    for(size_t i = 0; i < vec.size(); ++i)
    {
        file << vec[i];
        if(i != vec.size() - 1)
            file << ",";  // comma between values
    }

    file << "\n";  // new line at the end
    file.close();
}

const Eigen::Matrix3d Get_In_Position_Task::roll_rotation_nail_frame(const double &angle) const
{
  Eigen::Matrix3d res;
  res << 1,         0,                0,
         0,    cos(angle),    -sin(angle),
         0,    sin(angle),    cos(angle);
  return res;
}

const Eigen::Matrix3d Get_In_Position_Task::pitch_rotation_nail_frame(const double &angle) const
{
  Eigen::Matrix3d res;
  res << cos(angle),     0,       sin(angle),
               0,           1,            0,
         -sin(angle),     0,      cos(angle);
  return res;
  
}

const Eigen::Matrix3d Get_In_Position_Task::yaw_rotation_nail_frame(const double &angle) const
{
  Eigen::Matrix3d res;
  res<< cos(angle),   sin(angle),       0,
        -sin(angle),  cos(angle),       0,
              0,                0,           1;
  return res;
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
void Get_In_Position_Task::compare_configs(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_)
{
  std::vector<double> q_encoders = ctl_.robot().encoderValues();
  auto q_mbc = ctl_.robot().mbc().q;
  std::vector<double> q_dot_encoders = ctl_.robot().encoderVelocities();
  auto q_dot_mbc = ctl_.robot().mbc().alpha;
  auto ref_joint_order = ctl_.robot().refJointOrder();
  std::map<std::string, double> q_mbc_map;
  std::map<std::string, double> q_dot_mbc_map;
  std::map<std::string, double> q_encoders_map;
  std::map<std::string, double> q_dot_encoders_map;
  std::map<std::string, double> q_dot_encoders_derivative_map;
  std::map<std::string, double> q_dot_mbc_derivative_map;
  std::map<std::string, double> q_mbc_difference_map;
  std::map<std::string, double> q_encoders_difference_map;
  Eigen::VectorXd reorg_gradient = reorganized_gradient(gradient, ctl_);

  for(size_t i = 1; i < std::size(q_mbc); ++i)
  {
    if (q_mbc.at(i).empty()) {
      q_mbc_map[ctl_.robot().mb().joint(i).name()] = 0;
      q_dot_mbc_map[ctl_.robot().mb().joint(i).name()] = 0;
      q_mbc_difference_map[ctl_.robot().mb().joint(i).name()] = 0;
      q_dot_mbc_derivative_map[ctl_.robot().mb().joint(i).name()] = 0;

    }
    else {
      q_mbc_map[ctl_.robot().mb().joint(i).name()] = q_mbc.at(i).at(0);
      q_dot_mbc_map[ctl_.robot().mb().joint(i).name()] = q_dot_mbc.at(i).at(0);
      q_mbc_difference_map[ctl_.robot().mb().joint(i).name()] = q_mbc.at(i).at(0) - _old_mbc.q.at(i).at(0);
      q_dot_mbc_derivative_map[ctl_.robot().mb().joint(i).name()] = (q_mbc.at(i).at(0) - _old_mbc.q.at(i).at(0))/ctl_.solver().dt();
    }
  }

  for(size_t i = 0; i < std::size(q_dot_encoders); ++i)
  {
    q_encoders_map[ref_joint_order.at(i)] = q_encoders.at(i);
    q_dot_encoders_map[ref_joint_order.at(i)] = q_dot_encoders.at(i);
    q_encoders_difference_map[ref_joint_order.at(i)] = q_encoders.at(i) - _old_q_encoders.at(i);
    q_dot_encoders_derivative_map[ref_joint_order.at(i)] = (q_encoders.at(i) - _old_q_encoders.at(i))/ctl_.solver().dt();
  }

  mc_rtc::log::info("Q"); 

  for(const std::string &jointName : ref_joint_order)
  {
    mc_rtc::log::info("({}) q_mbc, q_encoders =  [{}, {}]", 
      jointName, 
      q_mbc_map.at(jointName),
      q_encoders_map.at(jointName)
      );
  }
  mc_rtc::log::info("---------------------"); 
  mc_rtc::log::info("Q_difference"); 

  for(const std::string &jointName : ref_joint_order)
  {
    mc_rtc::log::info("({}) q_mbc_difference, q_encoders_difference =  [{}, {}]", 
      jointName, 
      q_mbc_difference_map.at(jointName),
      q_encoders_difference_map.at(jointName)); 
  }
  mc_rtc::log::info("---------------------"); 
  mc_rtc::log::info("Q_DOT"); 

  uint8_t i = 0;
  for(const std::string &jointName : ref_joint_order)
  {
    mc_rtc::log::info("({}) q_dot_mbc, q_dot_mbc_der, q_dot_encoders, q_dot_encoders_der, grad_m =  [{}, {}, {}, {}, {}]", 
      jointName, 
      q_dot_mbc_map.at(jointName),
      q_dot_mbc_derivative_map.at(jointName),
      q_dot_encoders_map.at(jointName),
      q_dot_encoders_derivative_map.at(jointName),
      reorg_gradient(i, 0));
    ++i;
  }
  _old_q_encoders = q_encoders;
}
const double Get_In_Position_Task::emass_time_derivative_with_encoders(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_) const
{
  Eigen::VectorXd reorg_gradient = reorganized_gradient(gradient, ctl_);
  std::vector<double> q_dot_encoders = ctl_.robot().encoderVelocities();

  double dmdt = 0;
  for(size_t i = 0; i < std::size(q_dot_encoders); ++i)
  {
    dmdt += reorg_gradient(i, 0)*q_dot_encoders.at(i);
    // mc_rtc::log::info("({}) reorg_grad[{}] * qdot_encoders[{}] = {} * {}",ctl_.robot().refJointOrder().at(i), i, i,reorg_gradient(i, 0), q_dot_encoders.at(i) );
  }
  return dmdt;
}

const Eigen::VectorXd Get_In_Position_Task::reorganized_gradient(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_) const
{
  // Reorganize the gradient to multiply it by qdot from encoders
  Eigen::VectorXd res(std::size(ctl_.robot().encoderVelocities()), 1);
  res.setOnes();

  std::map<std::string, double> res_map;
  unsigned int j = 0;
  for(size_t i = 0; i < std::size(ctl_.robot().mbc().alpha); ++i)
  {
    if(std::size(ctl_.robot().mbc().alpha.at(i))!=1)
    {
      res_map[ctl_.robot().mb().joint(i).name()] = 0;
      continue;
    }
    res_map[ctl_.robot().mb().joint(i).name()] = gradient(j, 0);
    ++j;
  }
  for(size_t i = 0; i < std::size(ctl_.robot().refJointOrder()); ++i)
  {
    res(i, 0) = res_map.at(ctl_.robot().refJointOrder().at(i));
  }
  // mc_rtc::log::info("grad size = {}", std::size(gradient));
  // for(size_t i = 0; i < std::size(ctl_.robot().encoderVelocities()); ++i)
  // {
  //   if(i >= 0 && i < 6)
  //   {
  //     res(i, 0) = gradient(i+12, 0);
  //   }
  //   else if (i >= 6 && i < 12) 
  //   {  
  //     res(i, 0) = gradient(i, 0);
  //   }
  //   else if (i >= 12 && i < 17) 
  //   {
  //     res(i, 0) = gradient(i+6, 0);
  //   }
  //   else if (i >= 17 && i < 26) 
  //   {
  //     res(i, 0) = gradient(i+15, 0);
  //   }
  //   else if (i>= 26 && i < 35) 
  //   {
  //     res(i, 0) = 0;
  //   }
  //   else if (i >= 35 && i < 44) 
  //   {
  //     res(i, 0) = gradient(i-12, 0);
  //   }
  //   else if(i >= 44 && i < std::size(ctl_.robot().encoderVelocities()))
  //   {
  //     res(i, 0) = 0;
  //   }
  // }
  return res;
}

const double Get_In_Position_Task::emass_time_derivative_with_mbc_q_derivative(
  const Eigen::VectorXd &gradient, 
  mc_control::fsm::Controller & ctl_) const
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);

  double dmdt = 0;
  double delta_q_i = 0;
  unsigned int j = 0;
  std::vector<std::vector<double>> q = _new_mbc.q;
  std::vector<std::vector<double>> old_q = _old_mbc.q;
  for (size_t i = 0; i < std::size(q); ++i)
  {
    if (std::size(q.at(i)) != 1)
    {
      //Ignore floating base (size = 6) and empty dofs (size = 0)
      continue;
    }    
    delta_q_i = q.at(i).at(0) - old_q.at(i).at(0);
    dmdt += gradient(j,0) * (delta_q_i / ctl.solver().dt());
    // mc_rtc::log::info("({}) grad[{}] * dq[{}]/dt = {} * {}",ctl.robot().mb().joint(i).name(), j, i, gradient(j, 0), dqi/ctl.solver().dt());
    j+=1;
  }
  return dmdt;
}
const double Get_In_Position_Task::emass_time_derivative_with_mbc_alpha(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_) const
{
  double dmdt = 0;
  unsigned int j = 0;
  std::vector<std::vector<double>> qdot = _new_mbc.alpha;
  
  for (size_t i = 0; i < std::size(qdot); ++i)
  {
    if (std::size(qdot.at(i)) != 1)
    {
      //Ignore floating base and empty dofs
      continue;
    }    
    dmdt += gradient(j, 0)*qdot.at(i).at(0);

    j+=1;
  }

  return dmdt;
}

void Get_In_Position_Task::load_params()
{
  std::string magic_values_key = "magic_values";
  // _magic_posture_task_weight = _config(magic_values_key)("magic_posture_task_weight");
  // _magic_posture_task_stiffness = _config(magic_values_key)("magic_posture_task_stiffness");

  // _magic_effective_mass_maximization_task_weight = _config(magic_values_key)("magic_effective_mass_maximization_task_weight");

  // _magic_vector_orientation_task_weight = _config(magic_values_key)("magic_vector_orientation_task_weight");
  // _magic_vector_orientation_task_stiffness = _config(magic_values_key)("magic_vector_orientation_task_stiffness");
  // _magic_vector_orientation_task_damping = _config(magic_values_key).has("magic_vector_orientation_task_damping") ? _config(magic_values_key)("magic_vector_orientation_task_damping") : 2.0 * sqrt(_magic_vector_orientation_task_stiffness);

  // _magic_BSpline_task_dimweight_rx = _config(magic_values_key)("magic_BSpline_task_dimweight_rx");
  // _magic_BSpline_task_dimweight_ry = _config(magic_values_key)("magic_BSpline_task_dimweight_ry");
  // _magic_BSpline_task_dimweight_rz = _config(magic_values_key)("magic_BSpline_task_dimweight_rz");
  // _magic_BSpline_task_dimweight_tx = _config(magic_values_key)("magic_BSpline_task_dimweight_tx");
  // _magic_BSpline_task_dimweight_ty = _config(magic_values_key)("magic_BSpline_task_dimweight_ty");
  // _magic_BSpline_task_dimweight_tz = _config(magic_values_key)("magic_BSpline_task_dimweight_tz");
  // _magic_BSpline_max_duration = _config(magic_values_key)("magic_BSpline_max_duration");
  // _magic_BSpline_task_stiffness = _config(magic_values_key)("magic_BSpline_task_stiffness");
  // _magic_BSpline_task_damping = _config(magic_values_key).has("magic_BSpline_task_damping") ? _config(magic_values_key)("magic_BSpline_task_damping") : 2.0 * sqrt(_magic_BSpline_task_stiffness);
  // _magic_BSpline_task_weight = _config(magic_values_key)("magic_BSpline_task_weight");

  _logging_freq = _config(magic_values_key)("logging_freq");

  _gripper_task_weight = _config(magic_values_key)("gripper_task_weight");
  _gripper_task_min_stiffness = _config(magic_values_key)("gripper_task_min_stiffness");
  _gripper_task_max_stiffness = _config(magic_values_key)("gripper_task_max_stiffness");
  _gripper_task_goal_error = _config(magic_values_key)("gripper_task_goal_error");
  _gripper_task_K_scaling_factor = _config(magic_values_key)("gripper_task_s");

  _magic_hitting_target =  _config(magic_values_key)("hitting_target");
  // ------------------------ Loading init and start velocities, accelerations and jerks ---------------------------


  std::string curve_constraints_key = "curve_constraints";
  std::string linear_velocity_key = "linear_velocity";
  std::string linear_acceleration_key = "linear_acceleration";
  std::string x_key = "x";
  std::string y_key = "y";
  std::string z_key = "z";

  std::string init_key = "init";
  std::string end_key = "end";
  //_magic_normal_final_velocity = _config(curve_constraints_key)("magic_normal_final_velocity");

  // _constr.init_vel.x() = _config(curve_constraints_key)(linear_velocity_key)(0);
  // _constr.init_vel.y() = _config(curve_constraints_key)(linear_velocity_key)(1);
  // _constr.init_vel.z() = _config(curve_constraints_key)(linear_velocity_key)(2);

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

  ctl.logger().addLogEntry("GetInPoseTask_Bspline eval", this, [&, this]()
  {return ctl.bspline_eval;});
  ctl.logger().addLogEntry("GetInPoseTask_Bspline eval norm", this, [&, this]()
  {return ctl.bspline_eval_norm;});

  ctl.logger().addLogEntry("impact_detected",this,[&,this]()
  {return ctl.impact_detected*100;});

  ctl.logger().addLogEntry("impulsive constraint activation flag",this,[&,this]()
  {return ctl.impulsive_constraint_flag*100;});

  // ctl.logger().addLogEntry("Hitting_angle", this, [&, this]()
  // {return ctl.last_hitting_angle;});

  // ctl.logger().addLogEntry("Hitting_point", this, [&, this]()
  // {return ctl.last_hitting_point;});

  // ctl.logger().addLogEntry("Hitting_pointError", this, [&, this]()
  // {return ctl.last_hitting_point_error;});
  //
  // ctl.logger().removeLogEntry("Hitting_angle");
  // ctl.logger().removeLogEntry("Hitting_point");
  // ctl.logger().removeLogEntry("Hitting_pointError");
}

void Get_In_Position_Task::rm_logs(mc_control::fsm::Controller & ctl_)
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);

  ctl.logger().removeLogEntry("GetInPoseTask_Hammer tip reference bezier velocity [m/s]");
  ctl.logger().removeLogEntry("GetInPoseTask_Hammer tip reference bezier position [m]");
  ctl.logger().removeLogEntry("GetInPoseTask_Bspline tracking error [m]");
  ctl.logger().removeLogEntry("GetInPoseTask_Vector orientation error");
  ctl.logger().removeLogEntry("GetInPoseTask_Bspline eval");
  ctl.logger().removeLogEntry("GetInPoseTask_Bspline eval norm");
  ctl.logger().removeLogEntry("GetInPoseTask_angle eval");
  ctl.logger().removeLogEntry("impact_detected");

}

EXPORT_SINGLE_STATE("Get_In_Position_Task", Get_In_Position_Task)