//
//  Copyright 2020 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#include "ichthus_command_panel.hpp"

#include <QHBoxLayout>
#include <QString>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

inline std::string Bool2String(const bool var) { return var ? "True" : "False"; }

using std::placeholders::_1;

namespace rviz_plugins
{
IchthusCommandPanel::IchthusCommandPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // Autoware Engage Button
  auto_set_initial_pose_button_ptr_ = new QPushButton("Auto-Set Initial Pose");
  connect(auto_set_initial_pose_button_ptr_, SIGNAL(clicked()), SLOT(onClickAutoSetInitialPose()));

  // // Gate Mode Button
  // gate_mode_button_ptr_ = new QPushButton("Gate Mode");
  // connect(gate_mode_button_ptr_, SIGNAL(clicked()), SLOT(onClickGateMode()));

  // // Path Change Approval Button
  // path_change_approval_button_ptr_ = new QPushButton("Path Change Approval");
  // connect(path_change_approval_button_ptr_, SIGNAL(clicked()), SLOT(onClickPathChangeApproval()));

  // // Velocity Limit
  // velocity_limit_button_ptr_ = new QPushButton("Send Velocity Limit");
  // pub_velocity_limit_input_ = new QSpinBox();
  // pub_velocity_limit_input_->setRange(-100.0, 100.0);
  // pub_velocity_limit_input_->setValue(0.0);
  // pub_velocity_limit_input_->setSingleStep(5.0);
  // connect(velocity_limit_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickVelocityLimit()));

  // // Layout
  // auto * v_layout = new QVBoxLayout;
  // auto * gate_mode_path_change_approval_layout = new QHBoxLayout;
  // auto * velocity_limit_layout = new QHBoxLayout();
  // v_layout->addLayout(gate_layout);
  // v_layout->addLayout(selector_layout);
  // v_layout->addLayout(state_layout);
  // v_layout->addLayout(gear_layout);
  // v_layout->addLayout(engage_status_layout);
  // v_layout->addWidget(engage_button_ptr_);
  // v_layout->addLayout(engage_status_layout);
  // gate_mode_path_change_approval_layout->addWidget(gate_mode_button_ptr_);
  // gate_mode_path_change_approval_layout->addWidget(path_change_approval_button_ptr_);
  // v_layout->addLayout(gate_mode_path_change_approval_layout);
  // velocity_limit_layout->addWidget(velocity_limit_button_ptr_);
  // velocity_limit_layout->addWidget(pub_velocity_limit_input_);
  // velocity_limit_layout->addWidget(new QLabel("  [km/h]"));
  // v_layout->addLayout(velocity_limit_layout);
  // setLayout(v_layout);

  auto * v_layout = new QVBoxLayout;
  v_layout->addWidget(auto_set_initial_pose_button_ptr_);
  setLayout(v_layout);

}

void IchthusCommandPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  sub_gnss_pose_cov_ = 
    raw_node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/gnss_pose_cov", rclcpp::SensorDataQoS().keep_last(64),
    std::bind(&IchthusCommandPanel::callbackGnssPoseCov, this, std::placeholders::_1));

  pub_initialpose_ = raw_node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose3d", rclcpp::QoS(5));
}

void IchthusCommandPanel::onClickAutoSetInitialPose()
{
  if (latest_gnss_pose_cov_ != nullptr)
  {
    pub_initialpose_->publish(*latest_gnss_pose_cov_); 
  }
}

void IchthusCommandPanel::callbackGnssPoseCov(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  latest_gnss_pose_cov_ = msg;
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::IchthusCommandPanel, rviz_common::Panel)
