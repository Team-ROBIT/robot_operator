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
  qnode.updateTopic();
  init = true;
}

MainWindow::~MainWindow()
{
}

void MainWindow::slotUpdateImage(int num)
{
  QImage qImage((const unsigned char*)(qnode.img_raw[num].data), qnode.img_raw[num].cols, qnode.img_raw[num].rows,
                QImage::Format_RGB888);
  switch (num)
  {
    case 0:
      ui.img1->setPixmap(QPixmap::fromImage(qImage));
      break;
    case 1:
      ui.img2->setPixmap(QPixmap::fromImage(qImage));
      break;
    case 2:
      ui.img3->setPixmap(QPixmap::fromImage(qImage));
      break;
    default:
      break;
  }
}

void MainWindow::slotUpdateTopic()
{
  ui.topic_img2->clear();
  ui.topic_img3->clear();
  ui.topic_img2->addItems(qnode.topicList);
  ui.topic_img3->addItems(qnode.topicList);
}

void MainWindow::on_topic_img2_currentIndexChanged(int index)
{
  if (init)
  {
    QString topic = ui.topic_img2->currentText();
    qnode.img_topic[1] = topic.toStdString();
    qnode.changeTopic(1);
  }
}

void MainWindow::on_topic_img3_currentIndexChanged(int index)
{
  if (init)
  {
    QString topic = ui.topic_img3->currentText();
    qnode.img_topic[2] = topic.toStdString();
    qnode.changeTopic(2);
  }
}

void MainWindow::on_update_clicked()
{
  init = false;
  qnode.updateTopic();
  init = true;
}

}  // namespace robot_operator
