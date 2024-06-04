/**
 * @file /include/robot_operator/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef robot_operator_MAIN_WINDOW_H
#define robot_operator_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QImage>
#include <QString>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/default_plugin/map_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/visualization_frame.h"
#include "rviz/view_manager.h"
#include "rviz/config.h"
#include "rviz/yaml_config_reader.h"

#include <QVBoxLayout>

#include <ros/ros.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robot_operator
{
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = 0);
  ~MainWindow();

public Q_SLOTS:
  void slotUpdateImage(int num);
  void slotUpdateTopic();

  void on_topic_img1_currentIndexChanged(int index);
  void on_topic_img2_currentIndexChanged(int index);
  void on_topic_img3_currentIndexChanged(int index);
  void on_update_clicked();

private:
  Ui::MainWindowDesign ui;
  QNode qnode;

  bool init = false;

  // rviz simulation
  rviz::VisualizationFrame* rviz_frame_;
  rviz::VisualizationManager* rviz_manager_;
  rviz::ViewManager* view_manager_;

  // rviz slam
  rviz::VisualizationFrame* rviz_frame_2;
  rviz::VisualizationManager* rviz_manager_2;
  rviz::ViewManager* view_manager_2;
};

}  // namespace robot_operator

#endif  // robot_operator_MAIN_WINDOW_H
