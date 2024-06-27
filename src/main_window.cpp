/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/robot_operator/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_operator
{
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
{
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  qnode.init();

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(sigRcvImg(int)), this, SLOT(slotUpdateImage(int)));
  QObject::connect(&qnode, SIGNAL(sigReadTopic()), this, SLOT(slotUpdateTopic()));
  QObject::connect(&qnode, SIGNAL(sigUpdateState(float*, float*, bool*)), this,
                   SLOT(slotUpdateState(float*, float*, bool*)));
  qnode.updateTopic();
  init = true;

  // rviz simulation
  QString pathr1 = QString::fromStdString(qnode.rviz_path);
  rviz_frame_ = new rviz::VisualizationFrame();
  rviz_frame_->setParent(ui.rvizFrame_simulation);
  rviz_frame_->initialize(pathr1);
  rviz_frame_->setSplashPath("");
  rviz_frame_->setHideButtonVisibility(false);
  rviz_manager_ = rviz_frame_->getManager();
  QVBoxLayout* frameLayout = new QVBoxLayout(ui.rvizFrame_simulation);
  frameLayout->addWidget(rviz_frame_);
  rviz_frame_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  // rviz SLAM
  QString pathr2 = QString::fromStdString(qnode.rviz_path2);
  rviz_frame_2 = new rviz::VisualizationFrame();
  rviz_frame_2->setParent(ui.rvizFrame_slam);
  rviz_frame_2->initialize(pathr2);
  rviz_frame_2->setSplashPath("");
  rviz_frame_2->setHideButtonVisibility(false);
  rviz_manager_2 = rviz_frame_2->getManager();
  QVBoxLayout* frameLayout2 = new QVBoxLayout(ui.rvizFrame_slam);
  frameLayout2->addWidget(rviz_frame_2);
  rviz_frame_2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

MainWindow::~MainWindow()
{
}

void MainWindow::slotUpdateImage(int num)
{
  QImage qImage((const unsigned char*)(qnode.img_raw[num].data), qnode.img_raw[num].cols, qnode.img_raw[num].rows,
                QImage::Format_RGB888);
  QPixmap qPixmap = QPixmap::fromImage(qImage.rgbSwapped());

  QLabel* targetLabel = nullptr;
  QLabel* targetFps = nullptr;
  QLabel* targetSize = nullptr;

  switch (num)
  {
    case 0:
      targetLabel = ui.img1;
      targetFps = ui.fps_main;
      targetSize = ui.img_size_main;
      break;
    case 1:
      targetLabel = ui.img2;
      targetFps = ui.fps_2;
      targetSize = ui.img_size_2;
      break;
    case 2:
      targetLabel = ui.img3;
      targetFps = ui.fps_3;
      targetSize = ui.img_size_3;
      break;
    default:
      break;
  }

  if (targetLabel)
  {
    qPixmap = qPixmap.scaled(targetLabel->size(), Qt::KeepAspectRatio);
    targetLabel->setPixmap(qPixmap);
    targetLabel->setAlignment(Qt::AlignCenter);
  }

  targetFps->setText(QString::number(qnode.fps[num] / qnode.img_count[num]));
  targetSize->setText(QString::number(qnode.img_size[num]));
}

void MainWindow::slotUpdateTopic()
{
  ui.topic_img1->clear();
  ui.topic_img2->clear();
  ui.topic_img3->clear();
  ui.topic_img1->addItems(qnode.topicList);
  ui.topic_img2->addItems(qnode.topicList);
  ui.topic_img3->addItems(qnode.topicList);
}

void MainWindow::slotUpdateState(float* rpm, float* flipper, bool* flipper_status)
{
  ui.spd_L->setValue(rpm[0]);
  ui.spd_R->setValue(rpm[1]);
  ui.flipper_fl->setValue(flipper[0]);
  ui.flipper_fr->setValue(flipper[2]);
  ui.flipper_rl->setValue(flipper[1]);
  ui.flipper_rr->setValue(flipper[3]);
  setFlipperValueAndColor(ui.flipper_fl, flipper[0], flipper_status[0]);
  setFlipperValueAndColor(ui.flipper_rl, flipper[1], flipper_status[1]);
  setFlipperValueAndColor(ui.flipper_fr, flipper[2], flipper_status[2]);
  setFlipperValueAndColor(ui.flipper_rr, flipper[3], flipper_status[3]);
}

void MainWindow::on_topic_img1_currentIndexChanged(int index)
{
  if (init)
  {
    QString topic = ui.topic_img1->currentText();
    qnode.img_topic[0] = topic.toStdString();
    qnode.changeTopic(0);
    std::cout << "[robot_operator] Changed cam 1 topic to : " << topic.toStdString() << std::endl;
  }
}

void MainWindow::on_topic_img2_currentIndexChanged(int index)
{
  if (init)
  {
    QString topic = ui.topic_img2->currentText();
    qnode.img_topic[1] = topic.toStdString();
    qnode.changeTopic(1);
    std::cout << "[robot_operator] Changed cam 2 topic to : " << topic.toStdString() << std::endl;
  }
}

void MainWindow::on_topic_img3_currentIndexChanged(int index)
{
  if (init)
  {
    QString topic = ui.topic_img3->currentText();
    qnode.img_topic[2] = topic.toStdString();
    qnode.changeTopic(2);
    std::cout << "[robot_operator] Changed cam 3 topic to : " << topic.toStdString() << std::endl;
  }
}

void MainWindow::on_update_clicked()
{
  init = false;
  qnode.updateTopic();
  init = true;
}

void MainWindow::on_estop_clicked()
{
  qnode.emergencyStop();
}

void MainWindow::setFlipperValueAndColor(QDoubleSpinBox* spinBox, double value, bool status)
{
  spinBox->setValue(value);
  if (status)
  {
    spinBox->setStyleSheet("QDoubleSpinBox { color: green; }");
  }
  else
  {
    spinBox->setStyleSheet("QDoubleSpinBox { color: red; }");
  }
}

}  // namespace robot_operator
