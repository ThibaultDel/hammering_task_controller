#include "HammeringTaskNew.h"
#include <RBDyn/MultiBodyConfig.h>
// #include <mc_solver/DynamicsConstraint.h>


HammeringTaskNew::HammeringTaskNew(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{


  config_.load(config);
  datastore().make<std::string>("ControlMode", "Position");
  datastore().make<std::string>("Coriolis", "Yes"); 
  load_parameters();

  add_logs();

  nh = mc_rtc::ROSBridge::get_node_handle();
  // Not the cleanest but at leat mc_mujoco does not crash
  if(nh != nullptr)
  {
    subForce = nh->create_subscription<geometry_msgs::msg::Vector3Stamped>(
                  "/nail_force_sensor", 
                  1000,
                  std::bind(&HammeringTaskNew::nail_force_sensor_callback, this, std::placeholders::_1));
  }
  nail_rot = robot(nail_robot_name).frame(nail_frame_name).position().rotation();

  // Nail normal vector (n) expressed in world frame 
  nail_normal_vector_world_frame = (nail_rot.transpose()*normal_vector_nail_frame).normalized();

  // Store the initial posture of the robot
  // solver().addTask(postureTask);
  // postureTask->stiffness(100);
  contactConstraintSet = std::make_unique<mc_solver::ContactConstraint>(timeStep, mc_solver::ContactConstraint::ContactType::Acceleration);
  solver().addConstraintSet(contactConstraintSet);
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});

  // std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask = getPostureTask(robot().name());
  // base_posture_vector = FSMPostureTask->posture();

  // dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
  //   new mc_solver::DynamicsConstraint(
  //       robots(), 0, {0.1, 0.01, xsiOff_, m_, lambda_}, 0.9, true));
  // const std::array<double, 3> damping = {0.55, 0.30, 0.9};
  dynamicsConstraint = std::make_unique<mc_solver::DynamicsConstraint>(robots(), robot().robotIndex(), solver().dt(), _damping, _vp, false, true);
  solver().addConstraintSet(dynamicsConstraint);

  // Add impulse constraint
  Eigen::Vector3d normal_nail = robot(nail_robot_name).frame(nail_frame_name).position().rotation().col(2).eval();
  mc_rtc::log::info("the normal nail norm {}", normal_nail.norm());
  impulseConstraint = std::make_unique<mc_solver::ImpulseConstraint>(robots(), robot().robotIndex(), robot().frame(hammer_head_frame_name), normal_nail, _lambda_high, _lambda_low, _delta_t, _c_res, _dt_multi, logger());
  solver().addConstraintSet(impulseConstraint);

  // Load default configuration from robot module
  stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
  stabiConf.copMaxVel = {{3., 3., 3.}, {0.1, 0.1, 0.1}};
  // Create the stabilizer task
  stabilizerTask = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
            solver().robots(),
            solver().realRobots(),
            robot().robotIndex(),
            stabiConf.leftFootSurface,
            stabiConf.rightFootSurface,
            stabiConf.torsoBodyName,
            solver().dt());
  // Reset the task targets and default configuration
  stabilizerTask->reset();
  // Apply stabilizer configuration (optional, if not provided the default configuration from the RobotModule will be used)
  stabilizerTask->configure(stabiConf);
  // Set contacts (optional, the stabilizer will be configured in double support using the current foot pose as target for each contact by default)
  // stabilizerTask->setContacts({ContactState::Left, ContactState::Right});
  solver().addTask(stabilizerTask);
  stabilizerTask->torsoStiffness(_torso_task_stiffness);
  stabilizerTask->torsoWeight(_torso_task_weight);
  stabilizerTask->pelvisStiffness(_pelvis_task_stiffness);
  stabilizerTask->pelvisWeight(_pelvis_task_weight);
  stabilizerTask->dcmGains(_dcm_p, _dcm_i, _dcm_d);
  stabilizerTask->comStiffness(_com_stiffness);
  stabilizerTask->comWeight(_com_weight);
  stabilizerTask->contactWeight(_contact_task_weight);
  stabilizerTask->contactStiffness(_contact_stiffness);
  stabilizerTask->contactDamping(_contact_damping);
  stabilizerTask->copAdmittance(_contact_admittance);

  auto ext_wrench_conf = stabilizerTask->externalWrenchConfiguration();
  ext_wrench_conf.addExpectedCoMOffset = true;
  ext_wrench_conf.modifyCoMErr = true;
  ext_wrench_conf.modifyZMPErr = true;
  stabilizerTask->externalWrenchConfiguration(ext_wrench_conf);

  stabiConf = stabilizerTask->config();

  auto stab_config = stabilizerTask->config();

  auto & Active_tasks = solver().tasks();
  for (auto i:Active_tasks){
    mc_rtc::log::info("This controller has task: {} of type: {}", i->name(), i->type());
  }

  qd_previous=Eigen::VectorXd::Zero(robot().mb().nrDof());
  mc_rtc::log::success("HammeringTaskNew init done ");
}

bool HammeringTaskNew::run()
{
  // auto & bsRobot = comparisonRobots_->robot();
  // 1. Copy joint positions from the real robot
  // bsRobot.encoderValues(realRobot().encoderValues());

  // 2. Get the BodySensor data
  // Replace "FloatingBase" with the actual name of the sensor in your robot module
  // auto available_sensors = robot().bodySensors();
  // mc_rtc::log::info("The following bodysensors are available:");
  // for (const auto & sensor : available_sensors)
  // {
  //   mc_rtc::log::info(" - {}", sensor.name());
  // }
  // const auto & sensor = robot().bodySensor("FloatingBase");

  comparisonRobots_->robot().mbc().q = realRobot().mbc().q;

  // Manually set the floating base pose, velocity and acceleration in the world frame from the bodysensor
  comparisonRobots_->robot().posW(sva::PTransformd(floatingBaseSensor_.orientation(), floatingBaseSensor_.position()));
  comparisonRobots_->robot().velW(sva::MotionVecd(floatingBaseSensor_.angularVelocity(), floatingBaseSensor_.linearVelocity()));
  comparisonRobots_->robot().accW(sva::MotionVecd(floatingBaseSensor_.angularAcceleration(), floatingBaseSensor_.linearAcceleration()));

  // Update the kinematic tree
  comparisonRobots_->robot().forwardKinematics();
  comparisonRobots_->robot().forwardVelocity();
  comparisonRobots_->robot().forwardAcceleration();

  hammer_tip_actual_position_vector_realrobot = comparisonRobots_->robot().frame(hammer_head_frame_name).position().translation();

  hammer_tip_actual_position_vector = robot().frame(hammer_head_frame_name).position().translation();
  hammer_tip_actual_velocity_vector = robot().frame(hammer_head_frame_name).velocity().linear();

  hammer_tip_position_observer_error = hammer_tip_actual_position_vector - hammer_tip_actual_position_vector_realrobot;
  floating_base_position_observer_error = comparisonRobots_->robot().posW().translation() - robot().posW().translation();

  Eigen::VectorXd vec_joined(stabilizerTask->comeval().size() + stabilizerTask->contacteval().size());
  vec_joined << stabilizerTask->comeval(), stabilizerTask->contacteval();
  stabilizing_eval_norm = vec_joined.norm();
  stabilizing_speed_norm = stabilizerTask->speed().norm();

  com_eval = stabilizerTask->comeval();
  pelvis_eval = stabilizerTask->pelviseval();
  torso_eval = stabilizerTask->torsoeval();
  contacts_eval = stabilizerTask->contacteval();
  // contacts_eval = Eigen::VectorXd::Zero(6);

  rbd::Jacobian jac(robot().mb(), hammer_head_frame_name);
  Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot().mb(), robot().mbc());

  //Impulsive torque nail force method
  Eigen::MatrixXd full_world_frame_jacobian(6, robot().mb().nrDof());
  Eigen::MatrixXd & J_ = full_world_frame_jacobian;
  jac.fullJacobian(robot().mb(), world_frame_jacobian, J_);

  Eigen::MatrixXd P_n_sub = (nail_normal_vector_world_frame * nail_normal_vector_world_frame.transpose()).normalized();
  //Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);
  Eigen::MatrixXd linear_jacobian = J_.bottomRows(3);

  Impulsive_torque_f=linear_jacobian.transpose()*nail_force_vector;
  Impulsive_torque_projected_f=linear_jacobian.transpose()*P_n_sub*nail_force_vector;

  //Impulsive torque speed difference method

  P_n = Eigen::Matrix<double, 6, 6>::Zero();
  P_n.block<3,3>(3,3) = P_n_sub;

  qd=robot().encoderVelocities();

  qdm = Eigen::VectorXd::Zero(robot().mb().nrDof());

  qdm(0) = 0.0;
  qdm(1) = 0.0;
  qdm(2) = 0.0;
  qdm(3) = 0.0;
  qdm(4) = 0.0;
  qdm(5) = 0.0;
  qdm(6) = qd.at(6);//LCY
  qdm(7) = qd.at(7);//LCR
  qdm(8) = qd.at(8);//LCP
  qdm(9) = qd.at(9);//LKP
  qdm(10) = qd.at(10);//LAP
  qdm(11) = qd.at(11);//LAR
  qdm(12) = qd.at(0);//RCY
  qdm(13) = qd.at(1);//RCR
  qdm(14) = qd.at(2);//RCP
  qdm(15) = qd.at(3);//RKP
  qdm(16) = qd.at(4);//RAP
  qdm(17) = qd.at(5);//RAR
  qdm(18) = qd.at(12);//WP
  qdm(19) = qd.at(13);//WR
  qdm(20) = qd.at(14);//WY
  qdm(21) = qd.at(15);//HY
  qdm(22) = qd.at(16);//HP
  qdm(23) = qd.at(35);//LSC
  qdm(24) = qd.at(36);//LSP
  qdm(25) = qd.at(37);//LSR
  qdm(26) = qd.at(38);//LSY
  qdm(27) = qd.at(39);//LEP
  qdm(28) = qd.at(40);//LWRY
  qdm(29) = qd.at(41);//LWRR
  qdm(30) = qd.at(42);//LWRP
  qdm(31) = qd.at(43);//LHDY
  qdm(32) = qd.at(17);//RSC
  qdm(33) = qd.at(18);//RSP
  qdm(34) = qd.at(19);//RSR
  qdm(35) = qd.at(20);//RSY
  qdm(36) = qd.at(21);//REP
  qdm(37) = qd.at(22);//RWRY
  qdm(38) = qd.at(23);//RWRR
  qdm(39) = qd.at(24);//RWRP
  qdm(40) = qd.at(25);//RHDY


  effective_mass=compute_effective_mass_with_mbc(robot().mbc(),*this,nail_normal_vector_world_frame);
  Eigen::MatrixXd effective_mass_matrix=(J_*robot().tvmRobot().H().inverse()*J_.transpose()).inverse();
  tau_imp_true_speed=(J_.transpose()*effective_mass_matrix*P_n*J_)*(qd_previous-qdm)/_delta_t;
  tau_imp_act = (-1.f*(_c_res+1)/_delta_t)*J_.transpose()*effective_mass_matrix*P_n*J_*qdm;// use just for logging
  
  qd_previous = qdm;

  com_eval_norm = com_eval.norm();
  pelvis_eval_norm = pelvis_eval.norm();
  torso_eval_norm = torso_eval.norm();
  contacts_eval_norm = contacts_eval.norm();

  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop); // TODO: set to closedloop
}

void HammeringTaskNew::reset(const mc_control::ControllerResetData & reset_data)
{
  // auto robots = mc_rbdyn::loadRobot(robot().module());
  comparisonRobots_ = mc_rbdyn::loadRobot(robot().module());
  // comparisonRobots_ = std::make_shared<mc_rbdyn::Robot>(robots->robot(0).module(), robots->robot(0).name());
  mc_control::fsm::Controller::reset(reset_data);
}


double HammeringTaskNew::compute_effective_mass_with_mbc(
  rbd::MultiBodyConfig mbc, 
  mc_control::fsm::Controller & ctl_, 
  const Eigen::Vector3d &normal_vector){

  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);

  // If you dont put this line the gradient is 0 everywhere because M and J are not updating
  rbd::MultiBody robot_mb = ctl_.robot().mb();
  rbd::Jacobian jac(robot_mb, ctl.hammer_head_frame_name);
  Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot_mb, mbc);

  Eigen::MatrixXd full_world_frame_jacobian(6, ctl.robot().mb().nrDof());
  jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);

  rbd::ForwardDynamics fd(robot_mb);
  fd.computeH(robot_mb, mbc);
  Eigen::MatrixXd M = fd.H();
  

  Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);
  Eigen::Matrix3d LAMBDA = linear_jacobian*M.inverse()*linear_jacobian.transpose();
  end_effector_velocity=linear_jacobian*qdm;
  return 1/(normal_vector.transpose()*LAMBDA*normal_vector);
  }




void HammeringTaskNew::nail_force_sensor_callback(const std::shared_ptr<const geometry_msgs::msg::Vector3Stamped> &force)
{
  nail_force_vector.x() = force->vector.x;
  nail_force_vector.y() = force->vector.y;
  nail_force_vector.z() = force->vector.z;
}

void HammeringTaskNew::load_parameters()
{
  std::string global_controller = "global_controller_params";
  // ------------------------ Loading timestep ---------------------------
  std::string timestep_key = "timestep";

  // ------------------------ Loading gui parameters ---------------------------

  std::string gui_key = "gui";
  std::string stop_hammering_button_name_key = "stop_hammering_button_name";
  config_(global_controller)(gui_key)(stop_hammering_button_name_key, stop_hammering_button_name);

  // ------------------------ Loading quality of life parameters ---------------------------

  std::string qol_key = "quality_of_life";
  std::string jacobian_verbose_active_key = "jacobian_verbose_active";
  std::string bezier_curve_verbose_active_key = "bezier_curve_verbose_active";
  _bezier_curve_verbose_active = config_(global_controller)(qol_key)(bezier_curve_verbose_active_key);
  _jacobian_verbose_active = config_(global_controller)(qol_key)(jacobian_verbose_active_key);



  // ------------------------ Loading frames ---------------------------

  std::string frames_key = "frames";
  std::string hammerhead_frame_key = "Hammer_head";
  std::string nail_frame_key = "nail";
 config_(global_controller)(frames_key)(hammerhead_frame_key, hammer_head_frame_name);
 config_(global_controller)(frames_key)(nail_frame_key, nail_frame_name);

  // ------------------------ Loading magic values ---------------------------

  std::string magic_values_key = "magic_values";
  magic_force_threshold_nail = config_(global_controller)(magic_values_key)("magic_force_threshold_nail");
  magic_force_threshold_sensor = config_(global_controller)(magic_values_key)("magic_force_threshold_sensor");

  max_number_of_hits = config_(global_controller)(magic_values_key)("max_number_of_hits");

  // ------------------------ Loading constraint parameters ---------------------------
  _c_res  = config_(global_controller)(magic_values_key)("c_res");
  _lambda_high  = config_(global_controller)(magic_values_key)("lambda_high");
  _lambda_low = config_(global_controller)(magic_values_key)("lambda_low");
  _delta_t  = config_(global_controller)(magic_values_key)("delta_t");
  _dt_multi  = config_(global_controller)(magic_values_key)("impulsive_tau_limit_multiplier");
  _damping  = config_(global_controller)(magic_values_key)("damping");
  _vp  = config_(global_controller)(magic_values_key)("velocity_percentage");

  // ------------------------ Loading base parameters ---------------------------
  std::string robot_key = "hrp5_p";
  std::string posture_key = "posture";
  base_posture_stiffness = config_(robot_key)(posture_key)("stiffness");
  base_posture_weight = config_(robot_key)(posture_key)("weight");

  // ------------------------ Loading stabilizer parameters ---------------------------
  std::string global_control_param_key = "global_controller_params";
  std::string stabilizer_key = "stabilizer";

  _torso_task_stiffness = config_(global_control_param_key)(stabilizer_key)("torso")("stiffness");
  _torso_task_weight = config_(global_control_param_key)(stabilizer_key)("torso")("weight");
  _pelvis_task_stiffness = config_(global_control_param_key)(stabilizer_key)("pelvis")("stiffness");
  _pelvis_task_weight = config_(global_control_param_key)(stabilizer_key)("pelvis")("weight");
  _dcm_p = config_(global_control_param_key)(stabilizer_key)("dcm")("p");
  _dcm_i = config_(global_control_param_key)(stabilizer_key)("dcm")("i");
  _dcm_d = config_(global_control_param_key)(stabilizer_key)("dcm")("d");
  _com_stiffness = config_(global_control_param_key)(stabilizer_key)("com")("stiffness");
  _com_weight = config_(global_control_param_key)(stabilizer_key)("com")("weight");
  _contact_task_weight = config_(global_control_param_key)(stabilizer_key)("contact")("weight");
  _contact_stiffness = config_(global_control_param_key)(stabilizer_key)("contact")("stiffness");
  _contact_damping = config_(global_control_param_key)(stabilizer_key)("contact")("damping");
  _contact_admittance = config_(global_control_param_key)(stabilizer_key)("contact")("admittance");



}

void HammeringTaskNew::add_logs()
{
    logger().addLogEntry("Hammer tip velocity controller", this, [&,this]()
    {return end_effector_velocity;});

    logger().addLogEntry("ImpulsiveTorquePredicted_Actual", this, [&,this]()
    {return tau_imp_act;});

    logger().addLogEntry("ImpulsiveTorquesimulated_nailforce_fullVector", this, [&,this]()
    {return Impulsive_torque_f;});

    logger().addLogEntry("ImpulsiveTorquesimulated_nailforce_projected", this, [&,this]()
    {return Impulsive_torque_projected_f;});

    logger().addLogEntry("ImpulsiveTorquesimulated_speed",this,[&,this]
    {return tau_imp_true_speed;});

    logger().addLogEntry("Effective mass [kg]", this, [&, this]()
    {return effective_mass;});

    logger().addLogEntry("Effective mass derivative", this, [&, this]()
    {return effective_mass_diff;});

    logger().addLogEntry("Effective mass double derivative", this, [&, this]()
    {return effective_mass_diff_diff;});

    logger().addLogEntry("Effective mass diff checker", this, [&, this]()
    {return eff_mass_diff_checker;});

    logger().addLogEntry("Hammer tip velocity [m/s]", this, [&, this]()
    {return hammer_tip_actual_velocity_vector;});
      
    logger().addLogEntry("Hammer tip reference bezier velocity [m/s]", this, [&, this]()
    {return hammer_tip_reference_velocity_vector;});

    logger().addLogEntry("Hammer tip position [m]", this, [&, this]()
    {return hammer_tip_actual_position_vector;});

    logger().addLogEntry("Hammer tip position real robot [m]", this, [&, this]()
    {return hammer_tip_actual_position_vector_realrobot;});

    logger().addLogEntry("floating base body sensor_position", this, [&, this]()
    {return floatingBaseSensor_.position();});

    logger().addLogEntry("floating base body sensor_orientation", this, [&, this]()
    {return floatingBaseSensor_.orientation();});

    logger().addLogEntry("floating base body sensor_linearvelocity", this, [&, this]()
    {return floatingBaseSensor_.linearVelocity();});

    logger().addLogEntry("floating base body sensor_angularvelocity", this, [&, this]()
    {return floatingBaseSensor_.angularVelocity();});

    logger().addLogEntry("floating base body sensor_linearacceleration", this, [&, this]()
    {return floatingBaseSensor_.linearAcceleration();});

    logger().addLogEntry("floating base body sensor_angularacceleration", this, [&, this]()
    {return floatingBaseSensor_.angularAcceleration();});

    logger().addLogEntry("floating base observer error position", this, [&, this]()
    {return floating_base_position_observer_error;});

    logger().addLogEntry("Robot left hand force sensor", this, [&, this]()
    {return robot().forceSensor("LeftHandForceSensor").force();});

    logger().addLogEntry("Hammer tip observer error position", this, [&, this]()
    {return hammer_tip_position_observer_error;});

    // logger().addLogEntry("Hammer tip reference bezier position [m]", this, [&, this]()
    // {return hammer_tip_reference_position_vector;});

    // logger().addLogEntry("Bspline tracking error [m]", this, [&, this]()
    // {return bspline_tracking_error;});

    logger().addLogEntry("Projected momentum of hammer tip [kgm/s]", this, [&, this]()
    {return projected_momentum_of_hammer_tip;});

    // logger().addLogEntry("Vector orientation error", this, [&, this]()
    // {return vector_orientation_error;});

    logger().addLogEntry("Nail force sensor", this, [&, this]()
    {return nail_force_vector;});

    logger().addLogEntry("Nail force sensor norm", this, [&, this]()
    {return nail_force_vector.norm();});

    // logger().addLogEntry("Normal force applied to the nail", this, [&, this]()
    // {return vector_orientation_error;});

    logger().addLogEntry("bspline_active", this, [&, this]()
    {return 100*bspline_active_;});

    logger().addLogEntry("Stabilizing_speed_norm", this, [&, this]()
    {return stabilizing_speed_norm;});

    logger().addLogEntry("Stabilizing_eval_norm", this, [&, this]()
    {return stabilizing_eval_norm;});

    logger().addLogEntry("Stabilizing_com_eval_norm", this, [&, this]()
    {return com_eval_norm;});

    logger().addLogEntry("Stabilizing_torso_eval_norm", this, [&, this]()
    {return torso_eval_norm;});

    logger().addLogEntry("Stabilizing_pelvis_eval_norm", this, [&, this]()
    {return pelvis_eval_norm;});

    logger().addLogEntry("Stabilizing_contact_eval_norm", this, [&, this]()
    {return contacts_eval_norm;});

    logger().addLogEntry("Stabilizing_com_eval", this, [&, this]()
    {return com_eval;});

    logger().addLogEntry("Stabilizing_torso_eval", this, [&, this]()
    {return torso_eval;});

    logger().addLogEntry("Stabilizing_pelvis_eval", this, [&, this]()
    {return pelvis_eval;});

    logger().addLogEntry("Stabilizing_contact_eval", this, [&, this]()
    {return contacts_eval;});

    logger().addLogEntry("Completed_trajectories", this, [&, this]()
    {return trajectories_executed;});
    
    logger().addLogEntry("Number of hits", this, [&, this]()
    {return number_of_hits;});
}

