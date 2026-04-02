// Copyright 2025 Fan Yang
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#ifndef DRIVE_WIDGET_H
#define DRIVE_WIDGET_H

#include <math.h>
#include <stdio.h>

#include <QMouseEvent>
#include <QPainter>

#include <QWidget>

namespace teleop_rviz_plugin_plus {

class DriveWidget : public QWidget {
  Q_OBJECT
public:
  DriveWidget(QWidget *parent = 0);
  virtual void paintEvent(QPaintEvent *event);
  virtual void mouseMoveEvent(QMouseEvent *event);
  virtual void mousePressEvent(QMouseEvent *event);
  virtual void mouseReleaseEvent(QMouseEvent *event);
  virtual void leaveEvent(QEvent *event);
  virtual QSize sizeHint() const { return QSize(150, 150); }

Q_SIGNALS:
  void outputVelocity(float linear, float angular, bool pressed);

protected:
  void sendVelocitiesFromMouse(int x, int y, int width, int height);
  void stop();

  float linear_velocity_;
  float angular_velocity_;
  float linear_scale_;
  float angular_scale_;
  float x_mouse_, y_mouse_;
  bool mouse_pressed_;
};

} // namespace teleop_rviz_plugin_plus

#endif // DRIVE_WIDGET_H
