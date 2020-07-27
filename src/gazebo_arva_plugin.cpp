/*
 * @brief ARVA Plugin
 *
 * This plugin publishes ARVA data
 *
 * @author Lorenzo Gentilini (University of Bologna)
 * May, 2020
 */

#include <gazebo_arva_plugin.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(ArvaPlugin)

ArvaPlugin::ArvaPlugin():ModelPlugin(){;}

ArvaPlugin::~ArvaPlugin(){
  updateConnection_->~Connection();
}

void ArvaPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  #if GAZEBO_MAJOR_VERSION >= 9
    last_time_ = world_->SimTime();
  #else
    last_time_ = world_->GetSimTime();
  #endif

  // Default Params
  namespace_.clear();
  
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_arva_plugin] Please specify a robotNamespace.\n";

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_arva_plugin] Please specify a linkName.\n";

  // Initialize Node
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_arva_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  frame_id_ = link_name_;

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ArvaPlugin::OnUpdate, this, _1));

  // Publish Informations
  arva_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Arva>("~/" + model_->GetName() + "/arva", 10);
}

// This gets called by the world update start event
void ArvaPlugin::OnUpdate(const common::UpdateInfo&){
  #if GAZEBO_MAJOR_VERSION >= 9
    common::Time current_time = world_->SimTime();
  #else
    common::Time current_time = world_->GetSimTime();
  #endif

  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d T_W_I = link_->WorldPose();
  #else
    ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(link_->GetWorldPose());
  #endif
    
  ignition::math::Vector3d& P_W_I = T_W_I.Pos(); // Model Pose

  // Transmitter Consts
  double xt = X_REF;
  double yt = Y_REF;
  double zt = Z_REF;

  double theta  = THE_REF;
  double psi    = PSI_REF;
  double phi    = PHI_REF;

  /*
  double q0 = QW_REF;
  double q1 = QX_REF;
  double q2 = QY_REF;
  double q3 = QZ_REF;
  */

  // Convert Pose and Orientation Into Classical Vectors
  Eigen::Matrix<double, 3, 1> p(P_W_I.X() - xt, P_W_I.Y() - yt, P_W_I.Z() - zt);

  // Build Matrices Rt, Rr, A and e1
  Eigen::Matrix<double, 3, 3> Rt, Rx, Ry, Rz, A;
  Eigen::Matrix<double, 3, 1> e1 (1, 0, 0);

  Rx << 1, 0, 0,
        0, cos(phi), sin(phi),
        0, -sin(phi), cos(phi);

  Ry << cos(theta), 0, -sin(theta),
        0, 1, 0,
        sin(theta), 0, cos(theta);

  Rz << cos(psi), sin(psi), 0,
        -sin(psi), cos(psi), 0,
        0, 0, 1;

  // From Transmitter to Inertial
  // FIRST Rotation PHI - SECOND THETA - THIRD PSI
  Rt = Rz*Ry*Rx; 
  
  /*
  Rt << 1-2*pow(q2, 2)-2*pow(q3, 2), 2*q1*q2-2*q0*q3, 2*q1*q3+2*q0*q2,
        2*q1*q2+2*q0*q3, 1-2*pow(q1, 2)-2*pow(q3, 2), 2*q2*q3-2*q0*q1,
        2*q1*q3-2*q0*q2, 2*q2*q3+2*q0*q1, 1-2*pow(q1, 2)-2*pow(q2, 2);
  */

  A << 2*pow(p(0), 2)-pow(p(1), 2)-pow(p(2), 2), 3*p(0)*p(1), 3*p(0)*p(2),
       3*p(0)*p(1), 2*pow(p(1), 2)-pow(p(0), 2)-pow(p(2), 2), 3*p(1)*p(2),
       3*p(0)*p(2), 3*p(1)*p(2), 2*pow(p(2), 2)-pow(p(0), 2)-pow(p(1), 2);

  // Compute Magnetic Dipole
  Eigen::Matrix<double, 3, 1> H = (A*Rt*e1)/(4*M_PI*pow(p.norm(), 5));

  // Add Floor Noise
  static std::default_random_engine realGenerator;
  static std::default_random_engine intGenerator;
  std::uniform_real_distribution<double>  realDistribution(-1.0,1.0);
  std::uniform_int_distribution<int>      intDistribution(0,1);

  Eigen::Matrix<double, 3, 1> hNoise;
  double kNoise, rNoise, nNoise, xNoise, yNoise, zNoise;

  kNoise = 1.5420;
  rNoise = 80;
  nNoise = kNoise/(4*M_PI*pow(rNoise, 3));

  xNoise = realDistribution(realGenerator);
  yNoise = realDistribution(realGenerator)*sqrt(1 - pow(xNoise, 2));
  zNoise = pow(-1, intDistribution(intGenerator))*sqrt(1- pow(xNoise, 2) - pow(yNoise, 2));

  hNoise << xNoise, yNoise, zNoise;

  H += hNoise*nNoise*realDistribution(realGenerator);

  // Compute ARVA Function
  double y = 1/cbrt(H.norm());

  // publish Arva msg
  if((current_time - last_time_).Double() > arva_update_interval_){
    last_time_ = current_time;
  
    arva_msg.set_time_usec(current_time.Double()*1e6);
    arva_msg.set_arva_val(y);
    arva_pub_->Publish(arva_msg);
  }
}

} // namespace gazebo
