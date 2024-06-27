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
#include <QStringListModel>
#include <sensor_msgs/Image.h>
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QStringList>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include "mobile_base_msgs/STMtx.h"

#define MAIN_WIDTH 1600
#define MAIN_HEIGHT 900
#define SUB_WIDTH 800
#define SUB_HEIGHT 450

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

  std::vector<std::string> img_topic;
  std::vector<ros::Subscriber> img_sub_v;
  size_t img_size[3];
  ros::Time last_img_time[3];
  int img_count[3] = {
    0,
  };
  float fps[3] = {
    0,
  };
  cv::Mat img_raw[3];

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
  bool flipper_status[4] = {
    0,
  };
  bool light = false;

Q_SIGNALS:
  void rosShutdown();
  void sigRcvImg(int num);
  void sigReadTopic();
  void sigUpdateState();
  void sigUpdateJoyMode(int mode);

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
};

}  // namespace robot_operator

#endif /* robot_operator_QNODE_HPP_ */
