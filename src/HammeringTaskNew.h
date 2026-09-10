#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/gui/plot.h>


// BSplineTrajectoryTask and curve constraints
#include <mc_solver/DynamicsConstraint.h>
#include <mc_solver/ImpulseConstraint.h>
#include <mc_tasks/PostureTask.h>
#include <ndcurves/curve_constraint.h>

#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>
#include <mc_tasks/lipm_stabilizer/Contact.h>

// Ros node
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "rclcpp/rclcpp.hpp" //including ros2
#include <mc_rtc_ros/ros.h>
#include <vector>
#include "api.h"



typedef Eigen::Vector3d Point;
typedef Point point_t;
typedef ndcurves::curve_constraints<point_t> curve_constraints_t;
struct HammeringTaskNew_DLLAPI HammeringTaskNew : public mc_control::fsm::Controller
{
  public:
    HammeringTaskNew(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;


    bool impact_detected = false;
    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr subForce;
    mc_rtc::NodeHandlePtr nh; 

    // Hammer
    const Eigen::Vector3d normal_vector_to_align_in_hammerhead_frame = {1, 0, 0};

    // Nail
    Eigen::Matrix3d nail_rot;
    const Eigen::Vector3d normal_vector_nail_frame = {0, 0, 1};
    Eigen::Vector3d nail_normal_vector_world_frame = {0, 0, 0};
    //Eigen::Vector3d nail_force_vector = {0, 0, 0};
    //Eigen::Vector3d nail_force_vector_old = {0, 0, 0};
    Eigen::Matrix6d P_n; //projector nail n*n^T
    // Logs
    double effective_mass = 0.0f;
    double effective_mass_d = 0.0f;
    double effective_mass_dd = 0.0f;
    double eff_mass_diff_checker = 0.f;
    Eigen::Vector3d hammer_tip_actual_velocity_vector = {0, 0, 0};
    Eigen::Vector3d hammer_tip_actual_position_vector = {0, 0, 0};
    Eigen::Vector3d hammer_tip_actual_position_vector_realrobot = {0, 0, 0};
    Eigen::Vector3d hammer_tip_position_observer_error = {0, 0, 0};
    Eigen::Vector3d floating_base_position_observer_error = {0, 0, 0};
    
    Eigen::VectorXd full_world_frame_jacobian_log;

    Eigen::Vector3d hammer_tip_reference_velocity_vector = {0, 0, 0};
    Eigen::Vector3d hammer_tip_reference_position_vector = {0, 0, 0};
    Eigen::Vector3d bspline_tracking_error;
    Eigen::VectorXd bspline_eval;
    double bspline_eval_norm = 0.0f;
    bool bspline_active_ = false;
    double projected_momentum_of_hammer_tip = 0.0f;
    double vector_orientation_error = 0.0f;
    
    double time_step=0.0f;

    std::vector<double> qd;
    Eigen::VectorXd qdm;
    Eigen::VectorXd qd_previous;
    Eigen::VectorXd tau_imp_true_speed;
    Eigen::VectorXd tau_imp_true_force;
    Eigen::VectorXd tau_imp;
    Eigen::VectorXd tau_imp_act;
    Eigen::VectorXd tau_imp_previous;
    Eigen::VectorXd tau_imp_derivate;
    Eigen::VectorXd tau_imp_derivate_num;
    Eigen::VectorXd tau_imp_derivate_low_limit;
    Eigen::VectorXd tau_imp_derivate_high_limit;
    Eigen::VectorXd end_effector_velocity;

    std::vector<std::vector<double>> base_posture_vector;
    double base_posture_weight = 1.0f;
    double base_posture_stiffness = 1.0f;

    int trajectories_executed = 0;

    // constraints
    std::array<double, 3> _damping/* = {0.1, 0.01, 0.5}*/;
    double _vp;
    std::unique_ptr<mc_solver::DynamicsConstraint> dynamicsConstraint;
    Eigen::MatrixXd M_p;

    double _c_res;
    double _lambda_high;
    double _lambda_low;
    double _delta_t;
    double _dt_multi;
    double _tau_high_mulitplier=0.f;
    double _K=0;
    double _Activation_height=0;
    std::unique_ptr<mc_solver::ImpulseConstraint> impulseConstraint;


    std::unique_ptr<mc_solver::ContactConstraint> contactConstraintSet;

    // Tasks
    std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabilizerTask;
    mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stabiConf;

    double _torso_task_stiffness = 1.0f;
    double _torso_task_weight = 1.0f;
    double _pelvis_task_stiffness = 1.0f;
    double _pelvis_task_weight = 1.0f;
    Eigen::Vector2d _dcm_p = Eigen::Vector2d::Zero();
    Eigen::Vector2d _dcm_i = Eigen::Vector2d::Zero();
    Eigen::Vector2d _dcm_d = Eigen::Vector2d::Zero();
    Eigen::Vector3d _com_stiffness = Eigen::Vector3d::Zero();
    double _com_weight = 1.0f;
    double _contact_task_weight = 1.0f;
    sva::MotionVecd _contact_stiffness = sva::MotionVecd::Zero();
    sva::MotionVecd _contact_damping = sva::MotionVecd::Zero();
    Eigen::Vector2d _contact_admittance = Eigen::Vector2d::Zero();

    // Get_In_Position Task parameter for gui
    double _magic_posture_task_weight =1.0f;
    double _magic_posture_task_stiffness=1.0f;

    double _magic_BSpline_max_duration = 1.0f;
    double _magic_BSpline_task_stiffness = 1.0f; 
    double _magic_BSpline_task_damping = 1.0f;
    double _magic_BSpline_task_weight = 1.0f;
    double _magic_BSpline_task_dimweight_tx = 1.0f;
    double _magic_BSpline_task_dimweight_ty = 1.0f;
    double _magic_BSpline_task_dimweight_tz = 1.0f;
    double _magic_BSpline_task_dimweight_rx = 1.0f;
    double _magic_BSpline_task_dimweight_ry = 1.0f;
    double _magic_BSpline_task_dimweight_rz = 1.0f;
    Eigen::Vector6d dimweights = {0,0,0,0,0,0};
    Eigen::Vector3d _magic_init_vel = {0,0,0};
    double _magic_effective_mass_maximization_task_weight = 1.0f;

    double _magic_vector_orientation_task_weight = 1.0f;
    double _magic_vector_orientation_task_stiffness = 1.0f;
    double _magic_vector_orientation_task_damping = 1.0f;

    double _magic_normal_final_velocity = 1.0f;//
    Eigen::Vector3d _magic_final_velocity = {0, 0, 0};

    // ------------------------------ Parameters ---------------------------------------------
    // Parameters loaded in the load_parameters function, parameters are found in the HammeringTaskNew.in.yaml file
    // Don't ask me why there is a '.in' in the name of the file, I don't know 
    
    mc_rtc::Configuration config_;

    const std::string nail_robot_name = "nail";
    const std::string main_robot_name = "hrp5_p";
    const std::string hammer_head_frame_name = "Hammer_head";
    const std::string nail_frame_name = "nail";
    
    // quality of life
    
    bool _bezier_curve_verbose_active = false;
    bool _jacobian_verbose_active = false;
    
    // gui
    
    std::string stop_hammering_button_name = "undefined";
    std::string linear_constraint_button_name = "undefined";

    bool linear_impulsive_torque_ctr_flag=true;
    // Minimum force to detect an impact on the nail
    double magic_force_threshold_nail = 1;
    double magic_force_threshold_sensor = 1;


    double stabilizing_eval_norm;
    double stabilizing_speed_norm;

    Eigen::VectorXd com_eval;
    Eigen::VectorXd pelvis_eval;
    Eigen::VectorXd torso_eval;
    Eigen::VectorXd contacts_eval;

    double com_eval_norm;
    double pelvis_eval_norm;
    double torso_eval_norm;
    double contacts_eval_norm;

    double last_hitting_angle = 0.0f;
    double last_projected_momentum_of_hammer_tip = 0.0f;
    double last_hitting_angle_bodysensor = 0.0f;
    Eigen::Vector3d last_hitting_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_hitting_point_error_tilt = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_hitting_point_error_bodysensor = Eigen::Vector3d::Zero();
    double previous_hitting_angle = 0.0f;
    double previous_projected_momentum_of_hammer_tip = 0.0f;
    double previous_hitting_angle_bodysensor = 0.0f;
    Eigen::Vector3d previous_hitting_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_hitting_point_error_tilt = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_hitting_point_error_bodysensor = Eigen::Vector3d::Zero();
    bool hitting_data_to_log = false;
    bool hitting_logging_entry_to_remove = false;
    bool force_felt = false;
    bool hittingforce_data_to_log = false;
    bool hittingforce_logging_entry_to_remove = false;
    int number_of_hits = 0;
    Eigen::VectorXd Impulsive_torque_f;
    Eigen::VectorXd Impulsive_torque_projected_f;
    bool text_log_flag = false;
    bool impulsive_constraint_flag = false;
    std::vector<double> q_val;

    // Robot double:
    std::shared_ptr<mc_rbdyn::Robots> comparisonRobots_;
    // Helper to access it easily
    const mc_rbdyn::BodySensor & floatingBaseSensor_ = robot().bodySensor("FloatingBase");
    const std::vector<std::string> mass_maximization_active_joints = {
        "LCY" ,
        "LCR" ,
        "LCP" ,
        "LKP" ,
        "LAP" ,
        "LAR" ,
        "RCY" ,
        "RCR" ,
        "RCP" ,
        "RKP" ,
        "RAP" ,
        "RAR" ,
        "WP"  ,
        "WR"  ,
        "WY"  ,
        "HY"  ,
        "HP"  ,
        "LSC" ,
        "LSP" ,
        "LSR" ,
        "LSY" ,
        "LEP" ,
        "LWRY",
        "LWRR",
        "LWRP",
        "LHDY",
        "RSC" ,
        "RSP" ,
        "RSR" ,
        "RSY" ,
        "REP" ,
        "RWRY",
        "RWRR",
        "RWRP",
        "RHDY"
    };
    std::string selected_plot_joint_ = "LWRR";
    std::string selected_plot_mode_ = "Impulsive Torque";
    const std::vector<std::string> plot_modes_ = {"Impulsive Torque", "Derivative of Impulsive Torque"};
    int max_number_of_hits = 50;

    double compute_effective_mass_with_mbc(rbd::MultiBodyConfig mbc, 
                                                mc_control::fsm::Controller & ctl_, 
                                                const Eigen::Vector3d &normal_vector);

    double compute_effective_mass_d_with_mbc(rbd::MultiBodyConfig mbc, 
                                                mc_control::fsm::Controller & ctl_, 
                                                const Eigen::Vector3d &normal_vector,
                                                double effective_mass);
    double total_time_elapsed = 0;
    double plot_timer_ = 0.0;
    double plot_dt_ = 0.16; // Live plot update interval [s], loaded from YAML (default 0.16s / ~6 Hz)
    bool should_plot_tick_ = false;
  private:


    /**
    @brief Loads the parameters found in the HammeringTaskNew.in.yaml file
    */
    void load_parameters();

    /**
    @brief Adds some graphs to the logs of mc_log_ui
     */
    void add_logs();

    /**
    @brief Adds interface for live configuration modification element in Rviz and mc_mujoco
     */
    void addToGUI();

    /**
    @brief Store the force vector retrieved from the nail sensor plugin
    */
    void nail_force_sensor_callback(const std::shared_ptr<const geometry_msgs::msg::Vector3Stamped> &force);

};