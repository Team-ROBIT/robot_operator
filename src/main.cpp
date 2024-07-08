/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include <QDesktopWidget>
#include "../include/robot_operator/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  /*********************
  ** Qt
  **********************/
  QDesktopWidget* desktop = QApplication::desktop();
  int primaryScreenWidth = desktop->screenGeometry().width();
  int primaryScreenHeight = desktop->screenGeometry().height();
  QApplication app(argc, argv);
  robot_operator::MainWindow w(argc, argv);
  w.move(primaryScreenWidth, 0);

  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}
