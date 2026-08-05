#include "HammeringTaskNew_Initial.h"

#include <mc_rtc/clock.h>
#include <mc_rtc/logging.h>

#include "../HammeringTaskNew.h"

void HammeringTaskNew_Initial::configure(const mc_rtc::Configuration & config)
{
}

void HammeringTaskNew_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTaskNew &>(ctl_);

  // Creates a button to start the movement
  ctl.gui()->addElement({}, mc_rtc::gui::Button("Start hammering", [this]() { _positionning_hammer_clicked = true; }));
  // ctl.getPostureTask(ctl_.robot().name())->stiffness(100);
  // ctl.getPostureTask(ctl_.robot().name())->resetJointsSelector(ctl_.solver());
  // mc_rtc::log::info("posture dimweights are: {}",ctl.getPostureTask(ctl_.robot().name())->dimWeight());
  ctl.getPostureTask(ctl.robot().name())->stiffness(ctl.base_posture_stiffness);
  ctl.getPostureTask(ctl.robot().name())->weight(ctl.base_posture_weight);

  auto & Active_tasks = ctl.solver().tasks();
  for (auto i:Active_tasks){
    mc_rtc::log::info("This controller has task: {} of type: {}", i->name(), i->type());
  }

  ctl.logger().removeLogEntry("Hitting_angle");
  ctl.logger().removeLogEntry("Hitting_point");
  ctl.logger().removeLogEntry("Hitting_pointError");

  stabilizer_reset_done = false;

  mc_rtc::log::info("Starting Initial State");
  total_time_elapsed = 0.0f;
}

bool HammeringTaskNew_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
  // static_cast<r *>(constraint_.get())
  // static_cast<TVMImpulseConstraint *>(ctl.impulseConstraint->getConstraint().get())->impFunctionLow()->runUpdateA();
  // ctl.impulseConstraint->getConstraint().get()->impFunctionLow()->runUpdateB();
  // ctl.impulseConstraint->getConstraint().get()->impFunctionHigh()->runUpdateB();
  // ctl.impulseConstraint->getConstraint().get()->impFunctionLow()->runUpdateJacobian();
  // ctl.impulseConstraint->getConstraint().get()->impFunctionHigh()->runUpdateJacobian();
  total_time_elapsed += ctl_.solver().dt();
  // mc_rtc::log::info("posture dimweights are: {}",ctl.getPostureTask(ctl_.robot().name())->dimWeight());

  ctl.getPostureTask(ctl.robot().name())->stiffness(ctl.base_posture_stiffness);
  // // Stagger the buildup of the task stiffness for the transform task to not get a failing QP on startup of state
  // double error = ctl.getPostureTask(ctl.robot().name())->eval().norm();
  // double k_p = ctl.base_posture_stiffness;
  // if(first_iteration){
  //   first_instance_error = error;
  //   first_iteration = false;
  // }
  // // This function uses a scaled and shifted Hyperbolic Tangent (tanh) function to provide a
  // // non-linear, smooth, and bounded stiffness gain. The stiffness increases smoothly as the
  // // error approaches the goal, preventing abrupt, unstable jumps in control effort near
  // // the target while ensuring the stiffness is capped at K_max.
  // //
  // // The core relationship is designed such that:
  // // - As error -> first_instance_error (large), k_p -> min_stiffness (K_min).
  // // - As error -> goal_error (small), k_p -> max_stiffness (K_max).
  // k_p = ctl.base_posture_stiffness + (_posture_task_max_stiffness-ctl.base_posture_stiffness) * (
  //   (tanh((_posture_task_goal_error - error)/_posture_task_K_scaling_factor) - tanh((_posture_task_goal_error - first_instance_error)/_posture_task_K_scaling_factor)) /
  //   (1 - tanh((_posture_task_goal_error - first_instance_error)/_posture_task_K_scaling_factor)));
  //
  // //  mc_rtc::log::info("Current error is {} thus the stiffness is {}", error, k_p);
  // ctl.getPostureTask(ctl.robot().name())->stiffness(k_p);
  // mc_rtc::log::info("Posture stiffness set to {}", k_p);
  // if (total_time_elapsed > 3.f)
  // {
  //   mc_rtc::log::info("Stabilizing eval norm is {}", ctl.stabilizing_eval_norm);
  // }
  // mc_rtc::log::info("in initial state");

  if (ctl.hitting_logging_entry_to_remove)
  {
    ctl.logger().removeLogEntry("Hitting_angle");
    ctl.logger().removeLogEntry("Hitting_angle bodysensor");
    ctl.logger().removeLogEntry("Hitting_point");
    ctl.logger().removeLogEntry("Hitting_pointErrorTilt");
    ctl.logger().removeLogEntry("Hitting_pointErrorBodySensor");
    ctl.logger().removeLogEntry("Hitting_angle before hitting");
    ctl.logger().removeLogEntry("Hitting_angle bodysensor before hitting");
    ctl.logger().removeLogEntry("Hitting_point before hitting");
    ctl.logger().removeLogEntry("Hitting_pointErrorTilt before hitting");
    ctl.logger().removeLogEntry("Hitting_pointErrorBodySensor before hitting");
    ctl.logger().removeLogEntry("Hitting_ProjectedMomentum");
    ctl.logger().removeLogEntry("Hitting_ProjectedMomentum before hitting");
    ctl.hitting_logging_entry_to_remove = false;
  }

  if (ctl.hitting_data_to_log)
  {
    ctl.logger().addLogEntry("Hitting_angle", this, [&, this]()
    {return ctl.last_hitting_angle;});
    ctl.logger().addLogEntry("Hitting_angle bodysensor", this, [&, this]()
    {return ctl.last_hitting_angle_bodysensor;});
    ctl.logger().addLogEntry("Hitting_point", this, [&, this]()
    {return ctl.last_hitting_point;});
    ctl.logger().addLogEntry("Hitting_pointErrorTilt", this, [&, this]()
    {return ctl.last_hitting_point_error_tilt;});
    ctl.logger().addLogEntry("Hitting_pointErrorBodySensor", this, [&, this]()
    {return ctl.last_hitting_point_error_bodysensor;});
    ctl.logger().addLogEntry("Hitting_ProjectedMomentum", this, [&, this]()
    {return ctl.last_projected_momentum_of_hammer_tip;});
    ctl.logger().addLogEntry("Hitting_angle before hitting", this, [&, this]()
    {return ctl.previous_hitting_angle;});
    ctl.logger().addLogEntry("Hitting_angle bodysensor before hitting", this, [&, this]()
    {return ctl.previous_hitting_angle_bodysensor;});
    ctl.logger().addLogEntry("Hitting_point before hitting", this, [&, this]()
    {return ctl.previous_hitting_point;});
    ctl.logger().addLogEntry("Hitting_pointErrorTilt before hitting", this, [&, this]()
    {return ctl.previous_hitting_point_error_tilt;});
    ctl.logger().addLogEntry("Hitting_pointErrorBodySensor before hitting", this, [&, this]()
    {return ctl.previous_hitting_point_error_bodysensor;});
    ctl.logger().addLogEntry("Hitting_ProjectedMomentum before hitting", this, [&, this]()
    {return ctl.previous_projected_momentum_of_hammer_tip;});
    ctl.hitting_logging_entry_to_remove = true;
    ctl.hitting_data_to_log = false;
  }

  if (total_time_elapsed >= 0.2f)
  {
    if (ctl.hittingforce_logging_entry_to_remove)
    {
      ctl.logger().removeLogEntry("Hitting_force_felt");
      ctl.hittingforce_logging_entry_to_remove = false;
    }

    if (ctl.hittingforce_data_to_log)
    {
      ctl.logger().addLogEntry("Hitting_force_felt", this, [&, this]()
      {return ctl.force_felt;});
      ctl.hittingforce_logging_entry_to_remove = true;
      ctl.hittingforce_data_to_log = false;
    }
  }

  //if (ctl.number_of_hits >= ctl.max_number_of_hits && total_time_elapsed > 2.5f)
  //{
  //  mc_rtc::log::error_and_throw("This simulation is done to keep them all the same lengths");
  //}

  if ((_positionning_hammer_clicked) && ctl.stabilizing_eval_norm < 0.1 && ctl.number_of_hits < ctl.max_number_of_hits/* && ctl.pelvis_eval_norm < 0.05 && ctl.torso_eval_norm < 0.015*/)
  {
      output("BUTTON_CLICKED");
      return true;
  }
  return false;
}

void HammeringTaskNew_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
  ctl.force_felt = false;
  ctl_.gui()->removeElement({}, "Start hammering");
}

EXPORT_SINGLE_STATE("HammeringTaskNew_Initial", HammeringTaskNew_Initial)
