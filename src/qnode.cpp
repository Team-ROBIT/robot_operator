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

  start();
  return true;
}

void QNode::camCallback(const sensor_msgs::ImageConstPtr& msg, int num)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  img_raw[num] = cv_ptr->image;
  switch (num)
  {
    case 0:
      cv::resize(img_raw[num], img_raw[num], cv::Size(MAIN_WEIGHT, MAIN_HEIGHT), 0, 0, CV_INTER_LINEAR);
      break;

    default:
      cv::resize(img_raw[num], img_raw[num], cv::Size(SUB_WEIGHT, SUB_HEIGHT), 0, 0, CV_INTER_LINEAR);
      break;
  }
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

void QNode::changeTopic(int num)
{
  img_sub_v[num].shutdown();
  ros::NodeHandle n;
  img_sub_v[num] = n.subscribe<sensor_msgs::Image>(img_topic[num], 1, boost::bind(&QNode::camCallback, this, _1, num));
}

}  // namespace robot_operator
