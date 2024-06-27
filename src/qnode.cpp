/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/robot_operator/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_operator
{
/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv)
{
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc, init_argv, "robot_operator");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  readParams();

  for (int i = 0; i < img_topic.size(); i++)
  {
    img_sub_v.push_back(
        n.subscribe<sensor_msgs::Image>(img_topic[i], 1, boost::bind(&QNode::camCallback, this, _1, i)));
  }

  robot_status = n.subscribe<mobile_base_msgs::STMtx>("STM_tx_data", 1, &QNode::stmTxDataCallback, this);
  joy_mode = n.subscribe<std_msgs::Int32>("/joy_mode", 1, &QNode::joyModeCallback, this);
  eStop = n.advertise<std_msgs::Bool>("ESTOP", 1);

  rviz_path = ros::package::getPath("base_description");
  rviz_path = rviz_path + "/launch/urdf.rviz";
  rviz_path2 = ros::package::getPath("robot_operator");
  rviz_path2 = rviz_path2 + "/rviz/slam.rviz";
  start();
  return true;
}

void QNode::stmTxDataCallback(const mobile_base_msgs::STMtxConstPtr& data)
{
  rpm[0] = data->L_control.vel;
  rpm[1] = data->R_control.vel;
  for (int i = 0; i < 4; i++)
  {
    flipper[i] = data->flipper_control[i].target_pos;
    flipper_status[i] = data->flipper_control[i].init_req;
  }
  light = data->light;
  Q_EMIT sigUpdateState();
}

void QNode::joyModeCallback(const std_msgs::Int32ConstPtr& msg)
{
  sigUpdateJoyMode(msg->data);
}

void QNode::camCallback(const sensor_msgs::ImageConstPtr& msg, int num)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  img_raw[num] = cv_ptr->image;
  img_size[num] = msg->data.size();
  if (img_count[num] > 0)
  {
    ros::Duration duration = msg->header.stamp - last_img_time[num];
    double vFps = 1.0 / duration.toSec();
    fps[num] += vFps;
  }
  last_img_time[num] = msg->header.stamp;
  img_count[num]++;
  Q_EMIT sigRcvImg(num);
}

void QNode::run()
{
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::readParams()
{
  std::string data;
  ros::param::get("/robot_operator/cam1_topic", data);
  std::cout << "[robot_operator] Cam 1 topic : " << data.c_str() << std::endl;
  img_topic.push_back(data);
  ros::param::get("/robot_operator/cam2_topic", data);
  std::cout << "[robot_operator] Cam 2 topic : " << data.c_str() << std::endl;
  img_topic.push_back(data);
  ros::param::get("/robot_operator/cam3_topic", data);
  std::cout << "[robot_operator] Cam 3 topic : " << data.c_str() << std::endl;
  img_topic.push_back(data);
}

void QNode::updateTopic()
{
  ros::master::V_TopicInfo topics;
  if (ros::master::getTopics(topics))
  {
    topicList.clear();
    for (const auto& topic : topics)
    {
      if (topic.datatype == "sensor_msgs/Image")
      {
        topicList << QString::fromStdString(topic.name);
      }
    }
    Q_EMIT sigReadTopic();
    return;
  }
  else
  {
    ROS_ERROR("Failed to retrieve topics.");
    return;
  }
}

void QNode::emergencyStop()
{
  std_msgs::Bool msg;
  msg.data = true;
  eStop.publish(msg);
}

void QNode::changeTopic(int num)
{
  img_sub_v[num].shutdown();
  ros::NodeHandle n;
  img_sub_v[num] = n.subscribe<sensor_msgs::Image>(img_topic[num], 1, boost::bind(&QNode::camCallback, this, _1, num));
}

}  // namespace robot_operator
