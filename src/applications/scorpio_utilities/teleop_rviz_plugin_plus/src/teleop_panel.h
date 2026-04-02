// Copyright 2025 Fan Yang
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#endif

#include <stdio.h>

#include <QCheckBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

class QLineEdit;

namespace teleop_rviz_plugin_plus {

class DriveWidget;

class TeleopPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  TeleopPanel(QWidget *parent = 0);
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  void setVel(float linear_velocity_, float angular_velocity_,
              bool mouse_pressed_);

protected Q_SLOTS:
  void pressButton1();
  void pressButton2();
  void pressButton3();
  void pressButton4();
  void clickBox1(int val);
  void clickBox2(int val);
  void sendVel();

protected:
  DriveWidget *drive_widget_;

  // ROS2 uses smart pointers for publishers and subscribers
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr attemptable_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr update_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr read_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr save_publisher_;

  // Use shared pointer for the node
  rclcpp::Node::SharedPtr node_;

  QPushButton *push_button_1_;
  QPushButton *push_button_2_;
  QPushButton *push_button_3_;
  QPushButton *push_button_4_;
  QCheckBox *check_box_1_;
  QCheckBox *check_box_2_;

  float linear_velocity_;
  float angular_velocity_;
  bool mouse_pressed_;
  bool mouse_pressed_sent_;
};

} // namespace teleop_rviz_plugin_plus

#endif // TELEOP_PANEL_H
