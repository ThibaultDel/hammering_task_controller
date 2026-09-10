#include "HammeringTaskNew.h"
#include <RBDyn/MultiBodyConfig.h>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/Transform.h>
#include <mc_rtc/gui/plot.h>

HammeringTaskNew::HammeringTaskNew(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{


  config_.load(config);
  datastore().make<std::string>("ControlMode", "Position");
  datastore().make<std::string>("Coriolis", "Yes"); 
  load_parameters();

  add_logs();
  //nh = mc_rtc::ROSBridge::get_node_handle();
  // Not the cleanest but at leat mc_mujoco does not crash
  //if(nh != nullptr)
  //{
  //  subForce = nh->create_subscription<geometry_msgs::msg::Vector3Stamped>(
  //                "/nail_force_sensor", 
  //                1000,
  //                std::bind(&HammeringTaskNew::nail_force_sensor_callback, this, std::placeholders::_1));
  //}
  // Not the cleanest but at leat mc_mujoco does not crash
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
  mc_rtc::log::info("normal nail world frame {}", nail_normal_vector_world_frame);
  // impulseConstraint is initialized and added to solver in Get_In_Position_Task
  // impulseConstraint = std::make_unique<mc_solver::ImpulseConstraint>(robots(), robot().robotIndex(), robot().frame(hammer_head_frame_name), nail_normal_vector_world_frame, _lambda_high, _lambda_low, _delta_t, _c_res, _dt_multi, logger());
  //solver().addConstraintSet(impulseConstraint);

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

  auto stabconfig_ = stabilizerTask->config();

  auto & Active_tasks = solver().tasks();
  for (auto i:Active_tasks){
    mc_rtc::log::info("This controller has task: {} of type: {}", i->name(), i->type());
  }

  qd_previous=Eigen::VectorXd::Zero(robot().mb().nrDof());
  M_p=robot().tvmRobot().H();
  tau_imp_act = Eigen::VectorXd::Zero(robot().mb().nrDof());
  addToGUI();
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
  robot().forwardKinematics();
  robot().forwardVelocity();
  robot().forwardAcceleration();
  hammer_tip_actual_position_vector_realrobot = comparisonRobots_->robot().frame(hammer_head_frame_name).position().translation();

  hammer_tip_actual_position_vector = robot().frame(hammer_head_frame_name).position().translation();
  hammer_tip_actual_velocity_vector = robot().frame(hammer_head_frame_name).velocity().linear();

  hammer_tip_position_observer_error = hammer_tip_actual_position_vector - hammer_tip_actual_position_vector_realrobot;
  floating_base_position_observer_error = comparisonRobots_->robot().posW().translation() - robot().posW().translation();

  stabilizing_speed_norm = stabilizerTask->speed().norm();

  //Impulsive torque left arm force method
  rbd::Jacobian jac(robot().mb(), hammer_head_frame_name);
  Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot().mb(), robot().mbc());

  Eigen::MatrixXd full_world_frame_jacobian(6, robot().mb().nrDof());
  Eigen::MatrixXd & J_ = full_world_frame_jacobian;
  jac.fullJacobian(robot().mb(), world_frame_jacobian, J_);

  const auto & world_frame_jacobian_dot = jac.jacobianDot(robot().mb(), robot().mbc());
  Eigen::MatrixXd full_world_frame_jacobian_dot(6, robot().mb().nrDof());
  jac.fullJacobian(robot().mb() , world_frame_jacobian_dot, full_world_frame_jacobian_dot);
  Eigen::MatrixXd & J_d = full_world_frame_jacobian_dot;

  //Impulsive torque nail force method
  rbd::Jacobian jac_sensor(robot().mb(), "Larm_Link6"); //eft hand sensor associated joint
  Eigen::MatrixXd world_frame_jacobian_Larm_sensor = jac_sensor.jacobian(robot().mb(), robot().mbc());

  Eigen::MatrixXd J_Larm_sensor_(6, robot().mb().nrDof());
  jac_sensor.fullJacobian(robot().mb(), world_frame_jacobian_Larm_sensor, J_Larm_sensor_);

  Eigen::Matrix3d P_n_sub = (nail_normal_vector_world_frame * nail_normal_vector_world_frame.transpose()).normalized();
  //Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);
  Eigen::MatrixXd linear_jacobian = J_.bottomRows(3);

  //Impulsive torque speed difference method

  P_n = Eigen::Matrix<double, 6, 6>::Zero();
  P_n.block<3,3>(3,3) = P_n_sub;
  Eigen::VectorXd  q_d = tvm::dot(robot().tvmRobot().q(),1)->value();
  Eigen::VectorXd  q_dd = tvm::dot(robot().tvmRobot().q(),2)->value();

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
  effective_mass_d=compute_effective_mass_d_with_mbc(robot().mbc(),*this,nail_normal_vector_world_frame,effective_mass);
  tau_imp_true_speed=(J_.transpose()*effective_mass*P_n*J_)*solver().dt();

  Eigen::Matrix3d R = Eigen::AngleAxisd(-M_PI/4.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
  tau_imp_true_force=J_Larm_sensor_.transpose()*P_n_sub*(R*robot().forceSensor("LeftHandForceSensor").force());
  tau_imp = (-1.f*(_c_res+1)/_delta_t)*J_.transpose()*effective_mass*P_n*J_*q_d;
  tau_imp_act = (-1.f*(_c_res+1)/_delta_t)*J_.transpose()*effective_mass*P_n*J_*q_d;// use just for logging
  tau_imp_derivate = -(_c_res+1)/_delta_t*((J_d.transpose()*effective_mass*P_n*J_
  +J_.transpose()*effective_mass_d*P_n*J_
  +J_.transpose()*effective_mass*P_n*J_d)*q_d
  +(J_.transpose()*effective_mass*P_n*J_)*q_dd);
  tau_imp_derivate_num = (tau_imp_act-tau_imp_previous)/solver().dt();
  tau_imp_previous = tau_imp_act;
  //mc_solver::TVMImpulseConstraint* High_constraint = static_cast<mc_solver::TVMImpulseConstraint *>(impulseConstraint->getConstraint().get());
  //mc_solver::TVMImpulseConstraint* Low_constraint = static_cast<mc_solver::TVMImpulseConstraint *>(impulseConstraint->getConstraint().get());
  end_effector_velocity=linear_jacobian*q_d;

  tau_imp_derivate_low_limit=(robot().tvmRobot().limits().tl-tau_imp_act);
  tau_imp_derivate_high_limit=(robot().tvmRobot().limits().tu-tau_imp_act);

  qd_previous = qdm;
  total_time_elapsed += solver().dt();
  plot_timer_ += solver().dt();
  if(plot_timer_ >= plot_dt_)
  {
    should_plot_tick_ = true;
    plot_timer_ = 0.0;
  }
  else
  {
    should_plot_tick_ = false;
  }
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop); // TODO: set to closedloop
}

void HammeringTaskNew::reset(const mc_control::ControllerResetData & reset_data)
{
  total_time_elapsed = 0.0;
  plot_timer_ = 0.0;
  should_plot_tick_ = false;
  number_of_hits = 0;
  trajectories_executed = 0;
  impulsive_constraint_flag = false;
  force_felt = false;
  impact_detected = false;
  bspline_active_ = false;
  _Activation_height = 0.0;

  if(impulseConstraint)
  {
    solver().removeConstraintSet(*impulseConstraint);
    impulseConstraint.reset();
  }

  comparisonRobots_ = mc_rbdyn::loadRobot(robot().module());
  mc_control::fsm::Controller::reset(reset_data);

  this->resume("Hammer::HammeringFSM");
}

void HammeringTaskNew::addToGUI()
{
  this->gui()->addElement({"Hammering task"},
      mc_rtc::gui::ArrayInput("trajectory task weight"
      ,[this]() {return this->dimweights;}
      ,[this](const Eigen::Vector6d & weight) {this->dimweights = weight;}),
      mc_rtc::gui::ArrayInput("Orientation (weight stiffness damping)"
      ,[this]() { return Eigen::Vector3d{this->_magic_vector_orientation_task_weight,this->_magic_vector_orientation_task_stiffness,this->_magic_vector_orientation_task_damping};}
      ,[this](const Eigen::Vector3d & orientation_param) {_magic_vector_orientation_task_weight=orientation_param(0);
                                                        _magic_vector_orientation_task_stiffness=orientation_param(1);
                                                        _magic_vector_orientation_task_damping=orientation_param(2);}),
      mc_rtc::gui::ArrayInput("Bspline task (weight stiffness damping)"
      ,[this]() { return Eigen::Vector3d{this->_magic_BSpline_task_weight,this->_magic_BSpline_task_stiffness,this->_magic_BSpline_task_damping};}
      ,[this](const Eigen::Vector3d & Bspline_param) {_magic_BSpline_task_weight=Bspline_param(0);
                                                        _magic_BSpline_task_stiffness=Bspline_param(1);
                                                        _magic_BSpline_task_damping=Bspline_param(2);}),
      mc_rtc::gui::ArrayInput("Bspline params (duration final_velocity)"
      ,[this]() { return Eigen::Matrix<double, 2, 1>{this->_magic_BSpline_max_duration,this->_magic_normal_final_velocity};}
      ,[this](const Eigen::Matrix<double, 2, 1> & Bspline_param) {_magic_BSpline_max_duration=Bspline_param(0);
                                                        _magic_normal_final_velocity=Bspline_param(1);
                                                        _magic_final_velocity=_magic_final_velocity*_magic_normal_final_velocity;}),
      mc_rtc::gui::ArrayInput("Bspline init velocity (Vx Vy Vz)"
      ,[this]() { return this->_magic_init_vel;}
      ,[this](const Eigen::Vector3d & Bspline_init_velocity) {this->_magic_init_vel = Bspline_init_velocity;}),
      mc_rtc::gui::ArrayInput("Impulse Constraint(Cres dt multiplier velocityP lambdaH lambaL)"
      ,[this]() { return Eigen::Vector6d{this->_c_res,this->_delta_t,this->_dt_multi,this->_vp,this->_lambda_high,this->_lambda_low};}
      ,[this](const Eigen::Vector6d & Impusle_constraint_param) {_c_res=Impusle_constraint_param(0);
                                                        _delta_t=Impusle_constraint_param(1);
                                                        _dt_multi=Impusle_constraint_param(2);
                                                        _vp=Impusle_constraint_param(3);
                                                        _lambda_high=Impusle_constraint_param(4);
                                                        _lambda_low=Impusle_constraint_param(5);}),
      mc_rtc::gui::ArrayInput("Linear imptorque ctR parameters (Activation heigth,tau high multiplier,K)"
      ,[this]() { return Eigen::Vector3d{this->_Activation_height,this->_tau_high_mulitplier,this->_K};}
      ,[this](const Eigen::Vector3d & Linear_impulsive_constraint) {_Activation_height = Linear_impulsive_constraint(0);
                                                        _tau_high_mulitplier = Linear_impulsive_constraint(1),
                                                        _K = Linear_impulsive_constraint(2);}));
  this->gui()->addElement({},
    mc_rtc::gui::Checkbox(linear_constraint_button_name, [this]() { return linear_impulsive_torque_ctr_flag; }, [this]() { linear_impulsive_torque_ctr_flag = !linear_impulsive_torque_ctr_flag; }),
    mc_rtc::gui::ComboInput("Plot Joint", mass_maximization_active_joints,
      [this]() { return selected_plot_joint_; },
      [this](const std::string & j) { selected_plot_joint_ = j; }),
    mc_rtc::gui::NumberInput("Live plot interval [s]",
      [this]() { return plot_dt_; },
      [this](double dt) { plot_dt_ = std::max(0.01, dt); })
  );

  using Color = mc_rtc::gui::Color;
  using Style = mc_rtc::gui::plot::Style;
  using AxisConfig = mc_rtc::gui::plot::AxisConfiguration;

  auto get_dof = [this](const std::string & jname) -> int {
    for(size_t i = 0; i < mass_maximization_active_joints.size(); ++i)
    {
      if(mass_maximization_active_joints[i] == jname)
      {
        return static_cast<int>(i) + 6;
      }
    }
    return 29; // default to LWRR
  };

  auto get_upper = [this](int dof) -> double {
    if(impulseConstraint && impulseConstraint->TorqueHigherLimit().size() > dof)
    {
      return impulseConstraint->TorqueHigherLimit()(dof);
    }
    return robot().tvmRobot().limits().tu(dof) * _dt_multi;
  };

  auto get_lower = [this](int dof) -> double {
    if(impulseConstraint && impulseConstraint->TorqueLowerLimit().size() > dof)
    {
      return impulseConstraint->TorqueLowerLimit()(dof);
    }
    return robot().tvmRobot().limits().tl(dof) * _dt_multi;
  };

  auto get_out = [this](int dof) -> double {
    if(tau_imp_act.size() > dof)
    {
      return tau_imp_act(dof);
    }
    return 0.0;
  };

  auto make_curve = [this](const std::string & label, auto get_val, Color color, Style style) {
    return mc_rtc::gui::plot::XYChunk(
      label,
      [this, get_val](std::vector<std::array<double, 2>> & cache) {
        if(should_plot_tick_)
        {
          cache.push_back({total_time_elapsed, get_val()});
        }
      },
      color,
      style
    );
  };

  int dof_lwrr = get_dof("LWRR");
  int dof_lwrp = get_dof("LWRP");
  int dof_lwry = get_dof("LWRY");

  int dof_lep = get_dof("LEP");
  int dof_lsp = get_dof("LSP");
  int dof_lsr = get_dof("LSR");

  AxisConfig xAxis("t [s]");
  AxisConfig yAxis("Torque [N.m]");

  // Left Wrist (3 independent joint plots)
  this->gui()->addXYPlot(
    "Left Wrist: LWRR",
    xAxis, yAxis,
    make_curve("Output", [this, dof_lwrr, get_out]() { return get_out(dof_lwrr); }, Color::Red, Style::Solid),
    make_curve("Upper Limit", [this, dof_lwrr, get_upper]() { return get_upper(dof_lwrr); }, Color::Red, Style::Dotted),
    make_curve("Lower Limit", [this, dof_lwrr, get_lower]() { return get_lower(dof_lwrr); }, Color::Red, Style::Dotted)
  );
  this->gui()->addXYPlot(
    "Left Wrist: LWRP",
    xAxis, yAxis,
    make_curve("Output", [this, dof_lwrp, get_out]() { return get_out(dof_lwrp); }, Color::Blue, Style::Solid),
    make_curve("Upper Limit", [this, dof_lwrp, get_upper]() { return get_upper(dof_lwrp); }, Color::Blue, Style::Dotted),
    make_curve("Lower Limit", [this, dof_lwrp, get_lower]() { return get_lower(dof_lwrp); }, Color::Blue, Style::Dotted)
  );
  this->gui()->addXYPlot(
    "Left Wrist: LWRY",
    xAxis, yAxis,
    make_curve("Output", [this, dof_lwry, get_out]() { return get_out(dof_lwry); }, Color::Green, Style::Solid),
    make_curve("Upper Limit", [this, dof_lwry, get_upper]() { return get_upper(dof_lwry); }, Color::Green, Style::Dotted),
    make_curve("Lower Limit", [this, dof_lwry, get_lower]() { return get_lower(dof_lwry); }, Color::Green, Style::Dotted)
  );

  // Left Arm (3 independent joint plots)
  this->gui()->addXYPlot(
    "Left Arm: LEP",
    xAxis, yAxis,
    make_curve("Output", [this, dof_lep, get_out]() { return get_out(dof_lep); }, Color::Green, Style::Solid),
    make_curve("Upper Limit", [this, dof_lep, get_upper]() { return get_upper(dof_lep); }, Color::Green, Style::Dotted),
    make_curve("Lower Limit", [this, dof_lep, get_lower]() { return get_lower(dof_lep); }, Color::Green, Style::Dotted)
  );
  this->gui()->addXYPlot(
    "Left Arm: LSP",
    xAxis, yAxis,
    make_curve("Output", [this, dof_lsp, get_out]() { return get_out(dof_lsp); }, Color::Magenta, Style::Solid),
    make_curve("Upper Limit", [this, dof_lsp, get_upper]() { return get_upper(dof_lsp); }, Color::Magenta, Style::Dotted),
    make_curve("Lower Limit", [this, dof_lsp, get_lower]() { return get_lower(dof_lsp); }, Color::Magenta, Style::Dotted)
  );
  this->gui()->addXYPlot(
    "Left Arm: LSR",
    xAxis, yAxis,
    make_curve("Output", [this, dof_lsr, get_out]() { return get_out(dof_lsr); }, Color::Cyan, Style::Solid),
    make_curve("Upper Limit", [this, dof_lsr, get_upper]() { return get_upper(dof_lsr); }, Color::Cyan, Style::Dotted),
    make_curve("Lower Limit", [this, dof_lsr, get_lower]() { return get_lower(dof_lsr); }, Color::Cyan, Style::Dotted)
  );

  // Detailed Inspector for any selected joint
  this->gui()->addXYPlot(
    "Joint Details",
    xAxis, yAxis,
    make_curve("Predicted Output", [this, get_dof, get_out]() {
      return get_out(get_dof(selected_plot_joint_));
    }, Color::Green, Style::Solid),
    make_curve("Upper Limit", [this, get_dof, get_upper]() {
      return get_upper(get_dof(selected_plot_joint_));
    }, Color::Red, Style::Dotted),
    make_curve("Lower Limit", [this, get_dof, get_lower]() {
      return get_lower(get_dof(selected_plot_joint_));
    }, Color::Blue, Style::Dotted)
  );
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
  return 1/(normal_vector.transpose()*LAMBDA*normal_vector);
  }

double HammeringTaskNew::compute_effective_mass_d_with_mbc(
  rbd::MultiBodyConfig mbc, 
  mc_control::fsm::Controller & ctl_, 
  const Eigen::Vector3d &normal_vector,double effective_mass){

  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);

  // If you dont put this line the gradient is 0 everywhere because M and J are not updating
  rbd::MultiBody robot_mb = ctl_.robot().mb();
  rbd::Jacobian jac(robot_mb, ctl.hammer_head_frame_name);
  Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot_mb, mbc);

  Eigen::MatrixXd full_world_frame_jacobian(6, ctl.robot().mb().nrDof());
  jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);

  const auto & world_frame_jacobian_dot = jac.jacobianDot(robot_mb, mbc);
  Eigen::MatrixXd full_world_frame_jacobian_dot(6, robot().mb().nrDof());
  jac.fullJacobian(robot_mb, world_frame_jacobian_dot, full_world_frame_jacobian_dot);

  Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);
  Eigen::MatrixXd linear_jacobiand = full_world_frame_jacobian_dot.bottomRows(3);

  rbd::ForwardDynamics fd(robot_mb);
  fd.computeH(robot_mb, mbc);
  fd.computeC(robot_mb, mbc);
  Eigen::MatrixXd C = fd.C();
  Eigen::MatrixXd M = fd.H();
  Eigen::MatrixXd Mi = M.inverse();
  Eigen::MatrixXd M_d_ = (M-M_p)/solver().dt();
  M_p=M;
  double me_d =-1. * static_cast<double>(normal_vector.transpose() * (linear_jacobiand * Mi * linear_jacobian.transpose() -
  linear_jacobian * Mi * M_d_ * Mi * linear_jacobian.transpose() +
  linear_jacobian * Mi * linear_jacobiand.transpose()) * normal_vector) * effective_mass * effective_mass;
  if(abs(me_d)>1e5) return 0;
  else return me_d;
  }



void HammeringTaskNew::load_parameters()
{
  std::string global_controller = "global_controller_params";
  // ------------------------ Loading timestep -----------mc_----------------
  std::string timestep_key = "timestep";

  // ------------------------ Loading gui parameters ---------------------------

  std::string gui_key = "gui";
  std::string stop_hammering_button_name_key = "stop_hammering_button_name";
  std::string linear_constraint_button_name_key = "linear_constraint_button_name";
  config_(global_controller)(gui_key)(stop_hammering_button_name_key, stop_hammering_button_name);
  config_(global_controller)(gui_key)(linear_constraint_button_name_key, linear_constraint_button_name);
  if(config_(global_controller)(gui_key).has("plot_dt"))
  {
    config_(global_controller)(gui_key)("plot_dt", plot_dt_);
  }
  if(config_(global_controller)(gui_key).has("plot_joint"))
  {
    config_(global_controller)(gui_key)("plot_joint", selected_plot_joint_);
  }

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

  std::string hitting_constraint_paramater = "hitting_constraint_paramater";
  _c_res  = config_(global_controller)(hitting_constraint_paramater)("c_res");
  _lambda_high  = config_(global_controller)(hitting_constraint_paramater)("lambda_high");
  _lambda_low = config_(global_controller)(hitting_constraint_paramater)("lambda_low");
  _delta_t  = config_(global_controller)(hitting_constraint_paramater)("delta_t");
  _dt_multi  = config_(global_controller)(hitting_constraint_paramater)("impulsive_tau_limit_multiplier");
  _damping  = config_(global_controller)(hitting_constraint_paramater)("damping");
  _vp  = config_(global_controller)(hitting_constraint_paramater)("velocity_percentage");

  _Activation_height = config_(global_controller)(hitting_constraint_paramater)("Activation_height");;
  _tau_high_mulitplier = config_(global_controller)(hitting_constraint_paramater)("tau_high_mulitplier");
  _K = config_(global_controller)(hitting_constraint_paramater)("K");//that parameter correspond to a percentage. It is used to set the at what percent of the remaining distace between the target and the hammer the torque limit reaches the actual joint torque limit

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

  // ------------------------ Loading Bspline hammering parameters ---------------------------
  std::string curve_constraints_key = "curve_constraints";
  std::string linear_velocity_key = "linear_velocity";
  std::string linear_acceleration_key = "linear_acceleration";
  std::string hitting_tasks_paramater = "hitting_tasks_paramater";
  std::string x_key = "x";
  std::string y_key = "y";
  std::string z_key = "z";

  std::string init_key = "init";
  std::string end_key = "end";

  _magic_posture_task_weight = config_(global_control_param_key)(hitting_tasks_paramater)("magic_posture_task_weight");
  _magic_posture_task_stiffness = config_(global_control_param_key)(hitting_tasks_paramater)("magic_posture_task_stiffness");

  _magic_effective_mass_maximization_task_weight = config_(global_control_param_key)(hitting_tasks_paramater)("magic_effective_mass_maximization_task_weight");

  _magic_vector_orientation_task_weight = config_(global_control_param_key)(hitting_tasks_paramater)("magic_vector_orientation_task_weight");
  _magic_vector_orientation_task_stiffness = config_(global_control_param_key)(hitting_tasks_paramater)("magic_vector_orientation_task_stiffness");
  _magic_vector_orientation_task_damping = config_(global_control_param_key)(hitting_tasks_paramater).has("magic_vector_orientation_task_damping") ? config_(global_control_param_key)(hitting_tasks_paramater)("magic_vector_orientation_task_damping") : 2.0 * sqrt(_magic_vector_orientation_task_stiffness);

  _magic_BSpline_task_dimweight_rx = config_(global_control_param_key)(hitting_tasks_paramater)("magic_BSpline_task_dimweight_rx");
  _magic_BSpline_task_dimweight_ry = config_(global_control_param_key)(hitting_tasks_paramater)("magic_BSpline_task_dimweight_ry");
  _magic_BSpline_task_dimweight_rz = config_(global_control_param_key)(hitting_tasks_paramater)("magic_BSpline_task_dimweight_rz");
  _magic_BSpline_task_dimweight_tx = config_(global_control_param_key)(hitting_tasks_paramater)("magic_BSpline_task_dimweight_tx");
  _magic_BSpline_task_dimweight_ty = config_(global_control_param_key)(hitting_tasks_paramater)("magic_BSpline_task_dimweight_ty");
  _magic_BSpline_task_dimweight_tz = config_(global_control_param_key)(hitting_tasks_paramater)("magic_BSpline_task_dimweight_tz");
  dimweights(0) = _magic_BSpline_task_dimweight_rx; // dimweigth to be deleted
  dimweights(1) = _magic_BSpline_task_dimweight_ry;
  dimweights(2) = _magic_BSpline_task_dimweight_rz;
  dimweights(3) = _magic_BSpline_task_dimweight_tx;
  dimweights(4) = _magic_BSpline_task_dimweight_ty;
  dimweights(5) = _magic_BSpline_task_dimweight_tz;  
  _magic_BSpline_max_duration = config_(global_control_param_key)(hitting_tasks_paramater)("magic_BSpline_max_duration");
  _magic_BSpline_task_stiffness = config_(global_control_param_key)(hitting_tasks_paramater)("magic_BSpline_task_stiffness");
  _magic_BSpline_task_damping = config_(global_control_param_key)(hitting_tasks_paramater).has("magic_BSpline_task_damping") ? config_(global_control_param_key)(hitting_tasks_paramater)("magic_BSpline_task_damping") : 2.0 * sqrt(_magic_BSpline_task_stiffness);
  _magic_BSpline_task_weight = config_(global_control_param_key)(hitting_tasks_paramater)("magic_BSpline_task_weight");
  _magic_final_velocity = config_(global_control_param_key)(hitting_tasks_paramater)(curve_constraints_key)("magic_final_velocity");
  _magic_init_vel= config_(global_control_param_key)(hitting_tasks_paramater)(curve_constraints_key)("magic_init_velocity");
  
}

void HammeringTaskNew::add_logs()
{
    logger().addLogEntry("Hammer tip velocity controller", this, [&,this]()
    {return end_effector_velocity;});

    logger().addLogEntry("ImpulsiveTorquePredicted_Qptorque", this, [&,this]()
    {return tau_imp;});

    logger().addLogEntry("ImpulsiveTorquePredicted_Actual", this, [&,this]()
    {return tau_imp_act;});

    logger().addLogEntry("ImpulsiveTorquePredicted_derivative", this, [&,this]()
    {return tau_imp_derivate;});
    
    logger().addLogEntry("ImpulsiveTorquePredicted_numderivative", this, [&,this]()
    {return tau_imp_derivate_num;});

    logger().addLogEntry("ImpulsiveTorquePredicted_low_limit_derivative", this, [&,this]()
    {return tau_imp_derivate_low_limit;});

    logger().addLogEntry("ImpulsiveTorquePredicted_high_limit_derivative", this, [&,this]()
    {return tau_imp_derivate_high_limit;});

    logger().addLogEntry("ImpulsiveTorquesimulated_speed",this,[&,this]
    {return tau_imp_true_speed;});

    logger().addLogEntry("ImpulsiveTorquesimulated_force",this,[&,this]
    {return tau_imp_true_force;});

    logger().addLogEntry("Effective mass [kg]", this, [&, this]()
    {return effective_mass;});

    logger().addLogEntry("Effective mass derivative", this, [&, this]()
    {return effective_mass_d;});

    logger().addLogEntry("Effective mass double derivative", this, [&, this]()
    {return effective_mass_dd;});

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

    // logger().addLogEntry("Normal force applied to the nail", this, [&, this]()
    // {return vector_orientation_error;});

    logger().addLogEntry("bspline_active", this, [&, this]()
    {return 100*bspline_active_;});

    logger().addLogEntry("Stabilizing_speed_norm", this, [&, this]()
    {return stabilizing_speed_norm;});

    logger().addLogEntry("Stabilizing_eval_norm", this, [&, this]()
    {return stabilizing_eval_norm;});

    //logger().addLogEntry("Stabilizing_com_eval_norm", this, [&, this]()
    //{return com_eval_norm;});

    //logger().addLogEntry("Stabilizing_torso_eval_norm", this, [&, this]()
    //{return torso_eval_norm;});

    //logger().addLogEntry("Stabilizing_pelvis_eval_norm", this, [&, this]()
    //{return pelvis_eval_norm;});

    //logger().addLogEntry("Stabilizing_contact_eval_norm", this, [&, this]()
    //{return contacts_eval_norm;});

    //logger().addLogEntry("Stabilizing_com_eval", this, [&, this]()
    //{return com_eval;});

    //logger().addLogEntry("Stabilizing_torso_eval", this, [&, this]()
    //{return torso_eval;});

    //logger().addLogEntry("Stabilizing_pelvis_eval", this, [&, this]()
    //{return pelvis_eval;});

    //logger().addLogEntry("Stabilizing_contact_eval", this, [&, this]()
    //{return contacts_eval;});

    logger().addLogEntry("Completed_trajectories", this, [&, this]()
    {return trajectories_executed;});
    
    logger().addLogEntry("Number of hits", this, [&, this]()
    {return number_of_hits;});
}

