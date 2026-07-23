#include "Post_Impact_Task.h"
#include "../HammeringTaskNew.h"
#include <mc_rtc/logging.h>
#include <mc_tasks/PostureTask.h>
#include <memory>


void Post_Impact_Task::configure(const mc_rtc::Configuration & config)
{
    _config.load(config);
    // mc_rtc::log::info("Post_Impact_Task configure function called with config :  \n{}", config.dump(true, true));
    load_parameters();
}

void Post_Impact_Task::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);

    // _postureTask = std::make_shared<mc_tasks::PostureTask>(ctl.solver(),
    //                                                         ctl.robot().robotIndex());
    // _postureTask->posture(ctl.base_posture_vector);
    // _postureTask->stiffness(_magic_posture_task_stiffness);
    // _postureTask->weight(_magic_posture_task_weight);
    ctl.getPostureTask(ctl.robot().name())->stiffness(_magic_posture_task_stiffness);
    ctl.getPostureTask(ctl.robot().name())->weight(_magic_posture_task_weight);

    _target_velocity = -0.1f*ctl.nail_rot.transpose()*_magic_normal_final_velocity;
    _target_vel = sva::MotionVecd(Eigen::Vector3d::Zero(), _target_velocity);
    _nail_point = ctl.robots().robot(ctl.nail_robot_name).frame(ctl.nail_frame_name).position().translation() + Eigen::Vector3d(0.f, 0.f, 0.2f);
    // Eigen::Vector3d offset_from_nail = {0.f, 0.f, 0.1f};
    Eigen::Vector3d get_away_target =_nail_point + ctl.normal_vector_nail_frame*_post_impact_get_away_distance;
    _target_transform = sva::PTransformd(sva::RotX(M_PI)) * sva::PTransformd(sva::RotY(M_PI/2)) * sva::PTransformd(ctl.robots().robot(ctl.nail_robot_name).frame(ctl.nail_frame_name).position().rotation()) *sva::PTransformd(get_away_target) ;//* sva::PTransformd(Eigen::Vector3d(0.5, 0.2, 1));
    _transform_task = std::make_shared<mc_tasks::TransformTask>(ctl.robot().frame(ctl.hammer_head_frame_name), _transform_task_stiffness, _transform_task_weight);
    // _transform_task->reset();
    _transform_task->targetVel(_target_vel);

    _transform_task->target(_target_transform);
    _transform_task->setGains(_transform_task_stiffness, _transform_task_damping);

    // Eigen::Vector6d dimweights_transform_task = _transform_task->dimWeight();
    // // Remove the orientation part of the BSpline by setting the orientation weights to 0
    // dimweights_transform_task(0) = 0;
    // dimweights_transform_task(1) = 0;
    // dimweights_transform_task(2) = 0;
    // // Increase the weights on the x and y coordinates
    // dimweights_transform_task(3) = _magic_BSpline_task_dimweight_x;
    // dimweights_transform_task(4) = _magic_BSpline_task_dimweight_y;
    // dimweights_transform_task(5) = _magic_BSpline_task_dimweight_z;
    // _transform_task->dimWeight(dimweights_transform_task);
    ctl.solver().addTask(_transform_task);


    // _target_velocity = -1*ctl.nail_rot.transpose()*_magic_normal_final_velocity;
    // _target_vel = sva::MotionVecd(Eigen::Vector3d::Zero(), _target_velocity);
    // _nail_point = ctl.robots().robot(ctl.nail_robot_name).frame(ctl.nail_frame_name).position().translation();

    // _transform_task = std::make_shared<mc_tasks::TransformTask>(ctl.robot().frame(ctl.hammer_head_frame_name), _velocity_task_stiffness, _velocity_task_weight);
    // _transform_task->reset();
    // _transform_task->targetVel(_target_vel);
    // _target_transform = sva::PTransformd(sva::RotX(M_PI)) * sva::PTransformd(sva::RotY(M_PI/2)) * sva::PTransformd(ctl.robots().robot(ctl.nail_robot_name).frame(ctl.nail_frame_name).position().rotation()) *sva::PTransformd(_nail_point);//* sva::PTransformd(Eigen::Vector3d(0.5, 0.2, 1));
    //
    // _transform_task->target(_target_transform);
    // _transform_task->setGains(_velocity_task_stiffness, _velocity_task_stiffness);
    //
    // Eigen::Vector6d dimweights_transform_task = _transform_task->dimWeight();
    // // Remove the orientation part of the BSpline by setting the orientation weights to 0
    //
    // dimweights_transform_task(0) = 0;
    // dimweights_transform_task(1) = 0;
    // dimweights_transform_task(2) = 0;
    // // Increase the weights on the x and y coordinates
    // dimweights_transform_task(3) = _magic_BSpline_task_dimweight_x;
    // dimweights_transform_task(4) = _magic_BSpline_task_dimweight_y;
    // dimweights_transform_task(5) = _magic_BSpline_task_dimweight_z;
    // _transform_task->dimWeight(dimweights_transform_task);
    // ctl.solver().addTask(_transform_task);


    // ctl.solver().addTask(_postureTask);
}

bool Post_Impact_Task::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);

    // log_values(ctl_);

    duration += ctl.solver().dt();

    if (duration > 0.1f)
    {
        _transform_task->targetVel(sva::MotionVecd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
    }
    // Find a better condition than that
    if(/*ctl.getPostureTask(ctl.robot().name())->speed().norm() < 0.03*//*duration > 0.1f*/_transform_task->eval().norm() < 0.02f){
        output("STOP");
        return true;
    }
    // return _postureTask->eval().norm() < _magic_posture_task_epsilon && _postureTask->speed().norm() < 0.0003;
    return false;
}

void Post_Impact_Task::teardown(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
    // ctl.solver().removeTask(_postureTask);
    ctl.solver().removeTask(_transform_task);
}

void Post_Impact_Task::load_parameters()
{
    // --------------- Loading magic values ------------------------
    const std::string magic_values_key = "magic_values";
    _magic_posture_task_weight = _config(magic_values_key)("magic_posture_task_weight");
    _magic_posture_task_stiffness = _config(magic_values_key)("magic_posture_task_stiffness");
    _magic_posture_task_epsilon = _config(magic_values_key)("magic_posture_task_epsilon");

    _transform_task_weight = _config(magic_values_key)("transform_task_weight");
    _transform_task_stiffness = _config(magic_values_key)("transform_task_stiffness");
    _transform_task_damping = _config(magic_values_key)("transform_task_damping");


    _magic_BSpline_task_dimweight_x = _config(magic_values_key)("magic_BSpline_task_dimweight_x");
    _magic_BSpline_task_dimweight_y = _config(magic_values_key)("magic_BSpline_task_dimweight_y");
    _magic_BSpline_task_dimweight_z = _config(magic_values_key)("magic_BSpline_task_dimweight_z");

    std::string curve_constraints_key = "curve_constraints";
    _magic_normal_final_velocity = _config(curve_constraints_key)("magic_normal_final_velocity");


}

void Post_Impact_Task::log_values(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);

    ctl.effective_mass = ctl.compute_effective_mass_with_mbc(ctl.robot().mbc(),ctl,ctl.normal_vector_nail_frame);
    ctl.hammer_tip_actual_velocity_vector = ctl.robot().frame(ctl.hammer_head_frame_name).velocity().linear();
    ctl.hammer_tip_actual_position_vector = ctl.robot().frame(ctl.hammer_head_frame_name).position().translation();
    ctl.hammer_tip_reference_velocity_vector = {0, 0, 0};
    ctl.hammer_tip_reference_position_vector = {0, 0, 0};
    // ctl.hammer_tip_reference_position_vector = gripper_task->target().translation();
    ctl.projected_momentum_of_hammer_tip = 0;
    ctl.bspline_tracking_error = {0, 0, 0};

    Eigen::Matrix3d current_hammer_rotation = ctl.robot().frame(ctl.hammer_head_frame_name).position().rotation();
    ctl.vector_orientation_error = 0;
}

const std::vector<std::vector<double>> Post_Impact_Task::getHalfSittingPositionVector() const
{
    // Found in mc_hrp5p
    const std::map<std::string, double> halfSitting
    {
    {"RCY", 0.0},
    {"RCR", 0.0},
    {"RCP", -26.87},
    {"RKP", 50.0},
    {"RAP", -23.13},
    {"RAR", 0.0},
    {"LCY", 0.0},
    {"LCR", 0.0},
    {"LCP", -26.87},
    {"LKP", 50.0},
    {"LAP", -23.13},
    {"LAR", 0.0},
    {"WP",  0.0},
    {"WR",  0.0},
    {"WY",  0.0},
    {"HP",  0.0},
    {"HY",  0.0},
    {"RSC",  0.0},
    {"RSP",  60.0},
    {"RSR",  -20.0},
    {"RSY",  -5.0},
    {"REP",  -105.0},
    {"RWRY",  0.0},
    {"RWRR",  -40.0},
    {"RWRP",  0.0},
    {"RHDY",  0.0},
    {"LSC",  0.0},
    {"LSP",  60.0},
    {"LSR",  20.0},
    {"LSY",  5.0},
    {"LEP",  -105.0},
    {"LWRY",  0.0},
    {"LWRR",  40.0},
    {"LWRP",  0.0},
    {"LHDY",  0.0},
    {"LTMP",  -60.5}, // left thumb
    {"LTPIP",  0.0},
    {"LTDIP",  0.0},
    {"LMMP",  60.5}, // left middle
    {"LMPIP",  0.0},
    {"LMDIP",  0.0},
    {"LIMP",  60.5}, // left index
    {"LIPIP",  0.0},
    {"LIDIP",  0.0},
    {"RTMP",  60.5}, // right thumb   1.0559265
    {"RTPIP",  0.0},
    {"RTDIP",  0.0},
    {"RMMP",  -60.5}, // right middle -1.0559265
    {"RMPIP",  0.0},
    {"RMDIP",  0.0},
    {"RIMP",  -60.5}, // right index -1.0559265
    {"RIPIP",  0.0},
    {"RIDIP", 0.0},
    };
    std::vector<double> halfSittingVector = {};
    std::vector<std::vector<double>> res;
    double joint_value = 0.0f;
    for (const auto &pair : halfSitting)
    {
        joint_value = pair.second;
        halfSittingVector.push_back(joint_value);
    }
    res.push_back(halfSittingVector);
    return res;
}

EXPORT_SINGLE_STATE("Post_Impact_Task", Post_Impact_Task)
