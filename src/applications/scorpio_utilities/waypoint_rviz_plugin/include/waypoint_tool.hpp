// Copyright 2025 Fan Yang
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#ifndef WAYPOINT_TOOL_H
#define WAYPOINT_TOOL_H

#include <QObject>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/tool.hpp>

namespace rviz_common {
class DisplayContext;
namespace properties {
class StringProperty;
class QosProfileProperty;
} // namespace properties
} // namespace rviz_common

namespace waypoint_rviz_plugin {
class WaypointTool : public rviz_default_plugins::tools::PoseTool {
  Q_OBJECT
public:
  WaypointTool();

  ~WaypointTool() override;

  virtual void onInitialize() override;

protected:
  void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom);
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  float vehicle_z;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_joy_;

  rclcpp::Clock::SharedPtr clock_;

  rviz_common::properties::StringProperty *topic_property_;
  rviz_common::properties::QosProfileProperty *qos_profile_property_;

  rclcpp::QoS qos_profile_;
};
} // namespace waypoint_rviz_plugin

#endif // WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H
