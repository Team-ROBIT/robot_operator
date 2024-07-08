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
  delete timer10ms;
  delete timer1s;
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
  comm_sub = n.subscribe<std_msgs::Bool>("/comm_stat/robot", 1, &QNode::commStatusCallback, this);
  request_sub = n.subscribe<std_msgs::String>("/udp_request/robot", 1, &QNode::requestCallback, this);
  manipulator_sub = n.subscribe<std_msgs::Float32MultiArray>("/mani_angle", 1, &QNode::manipulatorCallback, this);
  circuit_sub = n.subscribe<mobile_base_msgs::STMrx>("STM_rx_data", 1, &QNode::circuitCallback, this);
  victim_sub = n.subscribe<std_msgs::String>("/victim_end", 1, &QNode::victimEndCallback, this);
  audio_sub = n.subscribe<audio_common_msgs::AudioData>("/audio/audio", 1, &QNode::audioCallback, this);

  eStop = n.advertise<std_msgs::Bool>("ESTOP", 1);
  comm_pub = n.advertise<std_msgs::Bool>("/comm_stat/operator", 1);
  request_pub = n.advertise<std_msgs::String>("/udp_request/operator", 1);
  manipulator_mission_pub = n.advertise<std_msgs::Int32>("/dex_mode", 1);
  victim_pub = n.advertise<std_msgs::String>("/victim_start", 1);
  joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

  rviz_path = ros::package::getPath("base_description");
  rviz_path = rviz_path + "/launch/urdf.rviz";
  rviz_path2 = ros::package::getPath("robot_operator");
  rviz_path2 = rviz_path2 + "/rviz/slam.rviz";

  timer10ms = new QTimer(this);
  timer1s = new QTimer(this);
  connect(timer10ms, &QTimer::timeout, this, &QNode::onTimer10ms);
  connect(timer1s, &QTimer::timeout, this, &QNode::onTimer1s);
  timer10ms->start(10);
  timer1s->start(1000);

  start();
  return true;
}

void QNode::audioCallback(const audio_common_msgs::AudioDataConstPtr& audio)
{
  xData.resize(audio->data.size());
  yData.resize(audio->data.size());
  for (size_t i = 0; i < audio->data.size(); ++i)
  {
    xData[i] = i;
    yData[i] = static_cast<double>(audio->data[i]);
  }
  Q_EMIT sigUpdateAudio();
}

void QNode::publishJoint(float joint[6], float flipper[4])
{
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();

  joint_state_msg.name = { "1axis_joint",     "5axis_joint",     "2axis_joint",     "3axis_joint",     "4axis_joint",
                           "flipper_joint_1", "flipper_joint_2", "flipper_joint_3", "flipper_joint_4", "6axis_joint" };
  joint_state_msg.position = { joint[0],   joint[4],   joint[1],   joint[2],   joint[3],
                               flipper[0], flipper[1], flipper[2], flipper[3], joint[5] };
  joint_state_msg.velocity = {};
  joint_state_msg.effort = {};
  joint_pub.publish(joint_state_msg);
}

void QNode::publishVictimBoxStart()
{
  ROS_INFO("Publishing Victim Box Start Topic.");
  std_msgs::String msg;
  msg.data = "start";
  victim_pub.publish(msg);
}

void QNode::victimEndCallback(const std_msgs::StringConstPtr& msg)
{
  if (msg->data == "end")
  {
    ROS_INFO("Victim Box End.");
    Q_EMIT sigUpdateVictimBox();
  }
}

void QNode::manipulatorMissionCallback(const std_msgs::Int32ConstPtr& msg)
{
  if (msg->data == 0)
  {
    ROS_INFO("Auto Manipulation End.");
    Q_EMIT sigUpdateManipulatorMission();
  }
}

void QNode::publishManipulatorMission(int mission_data)
{
  std_msgs::Int32 msg;
  msg.data = mission_data;
  ROS_INFO("Publishing Manipulator Auto Misson. Data : %d", mission_data);
  manipulator_mission_pub.publish(msg);
}

void QNode::circuitCallback(const mobile_base_msgs::STMrxConstPtr& data)
{
  battery[0] = data->battery[0];
  battery[1] = data->battery[1];

  Q_EMIT sigCircuitUpdate();
}

void QNode::manipulatorCallback(const std_msgs::Float32MultiArrayConstPtr& arr)
{
  for (int i = 0; i < 8; i++)
  {
    manipulator_angle[i] = arr->data[i];
  }

  float temp_mani[6] = {
    0,
  };
  for (int i = 0; i < 6; i++)
  {
    temp_mani[i] = manipulator_angle[i];
  }
  publishJoint(temp_mani, flipper_rad);
  Q_EMIT sigManipulatorUpdate();
}

void QNode::requestCallback(const std_msgs::StringConstPtr& req)
{
}

void QNode::publishRequest(std::string target, std::string request, std::string type)
{
  std::string temp = target + "/" + request + "/" + type;
  std_msgs::String msg;
  msg.data = temp;
  request_pub.publish(msg);
}

void QNode::commStatusCallback(const std_msgs::BoolConstPtr& stat)
{
  comm_cnt = 0;
  Q_EMIT sigStatusUpdate(true);
}

void QNode::onTimer10ms()
{
  std_msgs::Bool boolean_msg;
  boolean_msg.data = true;
  comm_pub.publish(boolean_msg);
}

void QNode::onTimer1s()
{
  comm_cnt++;
  if (comm_cnt >= 5)
  {
    ROS_ERROR("COMMUNICATION LOST. PLEASE CHECK YOUR CONNECTION.");
    Q_EMIT sigStatusUpdate(false);
  }
}

void QNode::stmTxDataCallback(const mobile_base_msgs::STMtxConstPtr& data)
{
  rpm[0] = data->L_control.vel;
  rpm[1] = data->R_control.vel;
  for (int i = 0; i < 4; i++)
  {
    flipper[i] = data->flipper_control[i].target_pos;
    flipper_rad[i] = Angles::toRAD(data->flipper_control[i].target_pos);
    flipper_status[i] = data->flipper_control[i].init_req;
  }
  light = data->light;
  float temp_mani[6] = {
    0,
  };
  for (int i = 0; i < 6; i++)
  {
    temp_mani[i] = manipulator_angle[i];
  }
  publishJoint(temp_mani, flipper_rad);
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
  ros::param::get("/robot_operator/cam4_topic", data);
  std::cout << "[robot_operator] Cam 4 topic : " << data.c_str() << std::endl;
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
