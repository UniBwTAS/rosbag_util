#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Time.h>

#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <rviz/panel.h>
#endif

namespace rosbag_panel
{

class RosbagPanel : public rviz::Panel
{
    Q_OBJECT
  public:
    RosbagPanel(QWidget* parent = 0);

  protected Q_SLOTS:
    void play();
    void pause();

  protected:
    QPushButton* play_button_;
    QPushButton* pause_button_;

  private:
    ros::ServiceClient service_client_;
    ros::NodeHandle nh_;
};

} // end namespace rosbag_panel
