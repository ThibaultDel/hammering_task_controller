#pragma once

#include <mc_control/fsm/State.h>
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/TransformTask.h>
#include <memory>

struct Post_Impact_Task : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl_) override;

  bool run(mc_control::fsm::Controller & ctl_) override;

  void log_values(mc_control::fsm::Controller & ctl_);

  void teardown(mc_control::fsm::Controller & ctl_) override;

  private:

    std::shared_ptr<mc_tasks::PostureTask> _postureTask;
    std::shared_ptr<mc_tasks::TransformTask> _transform_task;

    Eigen::Vector3d _target_velocity;
    sva::MotionVecd _target_vel;
    Eigen::Vector3d _magic_normal_final_velocity = {0, 0, 0};
    sva::PTransformd _target_transform;
    Eigen::Vector3d _nail_point;
    double _post_impact_get_away_distance=0.1f;
    double _transform_task_stiffness = 30.0f;
    double _transform_task_damping = 30.0f; //17
    double _transform_task_weight = 2000.0f;
    double _magic_BSpline_task_dimweight_x = 1.0f;
    double _magic_BSpline_task_dimweight_y = 1.0f;
    double _magic_BSpline_task_dimweight_z = 1.0f;

    double duration = 0.f;

    double previous_effective_mass = 0.f;

    /**
    @brief Returns the values needed to modify the posture task to the half sitting posture (the default posture)
     */
    const std::vector<std::vector<double>> getHalfSittingPositionVector() const;

    // ---------------- Parameters -----------------
    mc_rtc::Configuration _config;
    double _magic_posture_task_weight = 1.0f;
    double _magic_posture_task_stiffness = 1.0f;
    double _magic_posture_task_epsilon = 0.1f;


    void load_parameters();

};
