/**
 * @file /include/robot_operator/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robot_operator_QNODE_HPP_
#define robot_operator_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/package.h>
#endif
#include <string>
#include <QThread>
#include <QTimer>
#include <QStringListModel>
#include <sensor_msgs/Image.h>
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QStringList>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include "mobile_base_msgs/STMtx.h"
#include "mobile_base_msgs/STMrx.h"
#include <audio_common_msgs/AudioData.h>

#include "angle.hpp"

#define MAIN_WIDTH 1600
#define MAIN_HEIGHT 900
#define SUB_WIDTH 800
#define SUB_HEIGHT 450

#define MANIPULATOR_REQUEST "roslaunch manipulator24 mani.launch "
#define SLAM_REQUEST "roslaunch fast_lio mapping_velodyne_outside.launch "
#define CIRCUIT_REQUEST "roslaunch mobile_base_embedded_ros stm.launch "

namespace robot_operator
{
/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  int comm_cnt = 0;

  void publishRequest(std::string target, std::string request, std::string type);
  void publishManipulatorMission(int mission_data);
  void publishVictimBoxStart();

  std::vector<std::string> img_topic;
  std::vector<ros::Subscriber> img_sub_v;
  size_t img_size[4];
  ros::Time last_img_time[4];
  int img_count[4] = {
    0,
  };
  float fps[4] = {
    0,
  };
  cv::Mat img_raw[4];

  QStringList topicList;

  void changeTopic(int num);
  void emergencyStop();

  std::string rviz_path;
  std::string rviz_path2;

  float rpm[2] = {
    0,
  };
  float flipper[4] = {
    0,
  };
  float flipper_rad[4] = {
    0,
  };
  bool flipper_status[4] = {
    0,
  };
  bool light = false;

  float manipulator_angle[8] = {
    0,
  };

  float battery[2] = {
    0,
  };

  std::vector<double> xData, yData;

Q_SIGNALS:
  void rosShutdown();
  void sigRcvImg(int num);
  void sigReadTopic();
  void sigUpdateState();
  void sigUpdateJoyMode(int mode);
  void sigStatusUpdate(bool status);
  void sigManipulatorUpdate();
  void sigCircuitUpdate();
  void sigUpdateManipulatorMission();
  void sigUpdateVictimBox();
  void sigUpdateAudio();

public Q_SLOTS:
  void updateTopic();

private:
  int init_argc;
  char** init_argv;

  ros::Subscriber robot_status;
  void stmTxDataCallback(const mobile_base_msgs::STMtxConstPtr& data);

  ros::Subscriber joy_mode;
  void joyModeCallback(const std_msgs::Int32ConstPtr& msg);

  ros::Publisher eStop;

  void camCallback(const sensor_msgs::ImageConstPtr& msg, int num);
  void readParams();

  ros::Publisher comm_pub;
  ros::Subscriber comm_sub;
  void commStatusCallback(const std_msgs::BoolConstPtr& stat);

  ros::Publisher request_pub;
  ros::Subscriber request_sub;
  void requestCallback(const std_msgs::StringConstPtr& req);

  ros::Subscriber manipulator_sub;
  void manipulatorCallback(const std_msgs::Float32MultiArrayConstPtr& arr);

  ros::Subscriber circuit_sub;
  void circuitCallback(const mobile_base_msgs::STMrxConstPtr& data);

  ros::Publisher manipulator_mission_pub;
  ros::Subscriber manipulator_mission_sub;
  void manipulatorMissionCallback(const std_msgs::Int32ConstPtr& msg);

  ros::Publisher victim_pub;
  ros::Subscriber victim_sub;
  void victimEndCallback(const std_msgs::StringConstPtr& msg);

  ros::Publisher joint_pub;
  void publishJoint(float joint[6], float flipper[4]);

  ros::Subscriber audio_sub;
  void audioCallback(const audio_common_msgs::AudioDataConstPtr& audio);

  QTimer* timer10ms;
  QTimer* timer1s;

private slots:
  void onTimer10ms();
  void onTimer1s();
};

}  // namespace robot_operator

#endif /* robot_operator_QNODE_HPP_ */
