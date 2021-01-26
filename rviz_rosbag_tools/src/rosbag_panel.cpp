#include "rosbag_panel.h"
#include <QTimer>
#include <QtWidgets/QGridLayout>
#include <bits/stdc++.h>
#include <std_srvs/SetBool.h>

namespace rosbag_panel
{

RosbagPanel::RosbagPanel(QWidget* parent) : rviz::Panel(parent)
{
    QGridLayout* layout_obj = new QGridLayout(this);

    play_button_ = new QPushButton(QIcon::fromTheme("media-playback-start"), " Play");
    pause_button_ = new QPushButton(QIcon::fromTheme("media-playback-pause"), " Pause");

    layout_obj->addWidget(play_button_, 0, 2);
    layout_obj->addWidget(pause_button_, 0, 3);

    connect(play_button_, SIGNAL(clicked()), this, SLOT(play()));
    connect(pause_button_, SIGNAL(clicked()), this, SLOT(pause()));

    service_client_ = nh_.serviceClient<std_srvs::SetBool>("/rosbag_play/pause_playback");
}

void RosbagPanel::play()
{
    std_srvs::SetBool srv;
    srv.request.data = false;
    service_client_.call(srv);
}

void RosbagPanel::pause()
{
    std_srvs::SetBool srv;
    srv.request.data = true;
    service_client_.call(srv);
}

} // end namespace rosbag_panel

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rosbag_panel::RosbagPanel, rviz::Panel)
