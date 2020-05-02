/*
 * Copyright (C) 2012-2017 Open Source Robotics Foundation
 * Copyright (C) 2017-2018 PX4 Pro Development Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/**
 * @brief ARVA Plugin
 *
 * This plugin publishes ARVA data
 *
 * @author Lorenzo Gentilini
 */

#ifndef _GAZEBO_ARVA_PLUGIN_HH_
#define _GAZEBO_ARVA_PLUGIN_HH_

#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <queue>
#include <random>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <sdf/sdf.hh>
#include <common.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <Arva.pb.h>

#define X_REF 50
#define Y_REF -50
#define Z_REF 0

/*
#define QW_REF 1
#define QX_REF 0
#define QY_REF 0
#define QZ_REF 0
*/

#define THE_REF 10*M_PI/180
#define PSI_REF 155*M_PI/180
#define PHI_REF 0

namespace gazebo{
class GAZEBO_VISIBLE ArvaPlugin:public ModelPlugin{
public:
  ArvaPlugin();
  virtual ~ArvaPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo&);

private:
  std::string namespace_;
  std::default_random_engine random_generator_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr updateConnection_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr arva_pub_;

  sensor_msgs::msgs::Arva arva_msg;
  common::Time last_time_;

  std::string frame_id_;
  std::string link_name_; 

};     // class GAZEBO_VISIBLE ArvaPlugin
}      // namespace gazebo
#endif // _GAZEBO_ARVA_PLUGIN_HH_
