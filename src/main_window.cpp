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

  plot = new QwtPlot(this);
  ui.plotLayout->addWidget(plot);
  curve = new QwtPlotCurve();
  curve->attach(plot);

  qRegisterMetaType<std::vector<double>>("std::vector<double>");

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(sigRcvImg(int)), this, SLOT(slotUpdateImage(int)));
  QObject::connect(&qnode, SIGNAL(sigReadTopic()), this, SLOT(slotUpdateTopic()));
  QObject::connect(&qnode, SIGNAL(sigUpdateState()), this, SLOT(slotUpdateState()));
  QObject::connect(&qnode, SIGNAL(sigUpdateJoyMode(int)), this, SLOT(slotUpdateJoyMode(int)));
  QObject::connect(&qnode, SIGNAL(sigStatusUpdate(bool)), this, SLOT(slotStatusUpdate(bool)));
  QObject::connect(&qnode, SIGNAL(sigManipulatorUpdate()), this, SLOT(slotManipulatorUpdate()));
  QObject::connect(&qnode, SIGNAL(sigCircuitUpdate()), this, SLOT(slotUpdateCircuit()));
  QObject::connect(&qnode, SIGNAL(sigUpdateManipulatorMission()), this, SLOT(slotUpdateManipulatorMission()));
  QObject::connect(&qnode, SIGNAL(sigUpdateVictimBox()), this, SLOT(slotUpdateVictimBox()));
  QObject::connect(&qnode, SIGNAL(sigUpdateAudio()), this, SLOT(slotUpdateAudio()));
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
  delete plot;
  delete curve;
}

void MainWindow::slotUpdateAudio()
{
  QPen pen(Qt::green);
  pen.setWidth(2);
  if (qnode.xData.size() > 0 && qnode.yData.size() > 0)
  {
    curve->setPen(pen);
    curve->setSamples(qnode.xData.data(), qnode.yData.data(), qnode.yData.size());
    plot->replot();
  }
}

void MainWindow::generateSineWave(short* buffer, int sample_rate, int frequency, int amplitude, int samples)
{
  double increment = 2.0 * M_PI * frequency / sample_rate;
  for (int i = 0; i < samples; ++i)
  {
    buffer[i] = amplitude * sin(increment * i);
  }
}

void MainWindow::playSiren(int duration)
{
  snd_pcm_t* pcm_handle;
  snd_pcm_hw_params_t* params;
  int dir;
  unsigned int sample_rate = SAMPLE_RATE;
  snd_pcm_uframes_t frames = 32;
  int rc;

  rc = snd_pcm_open(&pcm_handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
  if (rc < 0)
  {
    std::cerr << "Unable to open PCM device: " << snd_strerror(rc) << std::endl;
    return;
  }

  snd_pcm_hw_params_alloca(&params);
  snd_pcm_hw_params_any(pcm_handle, params);
  snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
  snd_pcm_hw_params_set_format(pcm_handle, params, SND_PCM_FORMAT_S16_LE);
  snd_pcm_hw_params_set_channels(pcm_handle, params, 1);
  snd_pcm_hw_params_set_rate_near(pcm_handle, params, &sample_rate, &dir);
  snd_pcm_hw_params_set_period_size_near(pcm_handle, params, &frames, &dir);

  rc = snd_pcm_hw_params(pcm_handle, params);
  if (rc < 0)
  {
    std::cerr << "Unable to set HW parameters: " << snd_strerror(rc) << std::endl;
    return;
  }

  int size = frames * 2;  // 2 bytes/sample, 1 channel
  short* buffer = (short*)malloc(size);
  int frequency = FREQUENCY_LOW;

  for (int t = 0; t < duration * SAMPLE_RATE / frames; ++t)
  {
    if (t % (SAMPLE_RATE / frames) < (SAMPLE_RATE / frames) / 2)
    {
      frequency = FREQUENCY_LOW +
                  (FREQUENCY_HIGH - FREQUENCY_LOW) * (t % (SAMPLE_RATE / frames)) / ((SAMPLE_RATE / frames) / 2);
    }
    else
    {
      frequency = FREQUENCY_HIGH - (FREQUENCY_HIGH - FREQUENCY_LOW) *
                                       (t % (SAMPLE_RATE / frames) - (SAMPLE_RATE / frames) / 2) /
                                       ((SAMPLE_RATE / frames) / 2);
    }
    generateSineWave(buffer, sample_rate, frequency, AMPLITUDE, frames);
    rc = snd_pcm_writei(pcm_handle, buffer, frames);
    if (rc == -EPIPE)
    {
      std::cerr << "Underrun occurred" << std::endl;
      snd_pcm_prepare(pcm_handle);
    }
    else if (rc < 0)
    {
      std::cerr << "Error writing to PCM device: " << snd_strerror(rc) << std::endl;
    }
  }

  snd_pcm_drain(pcm_handle);
  snd_pcm_close(pcm_handle);
  free(buffer);
}

void MainWindow::slotUpdateVictimBox()
{
  ui.btn_victim_box->setEnabled(true);
  ui.btn_auto_manipulator->setEnabled(true);
  ui.btn_victim_box->setStyleSheet("QPushButton { color : white; }");
}

void MainWindow::on_btn_victim_box_clicked()
{
  qnode.publishVictimBoxStart();
  ui.btn_victim_box->setStyleSheet("QPushButton { color : green; }");
  ui.btn_victim_box->setEnabled(false);
  ui.btn_auto_manipulator->setEnabled(false);
}

void MainWindow::slotUpdateManipulatorMission()
{
  ui.linear->setChecked(false);
  ui.omni->setChecked(false);
  ui.touch->setChecked(false);
  ui.inspect->setChecked(false);
  ui.btn_victim_box->setEnabled(true);
  ui.btn_auto_manipulator->setEnabled(true);
  ui.btn_auto_manipulator->setStyleSheet("QPushButton { color : white; }");
}

void MainWindow::on_omni_clicked()
{
  if (ui.linear->isChecked())
  {
    ui.linear->setChecked(false);
  }
}

void MainWindow::on_linear_clicked()
{
  if (ui.omni->isChecked())
  {
    ui.omni->setChecked(false);
  }
}

void MainWindow::on_inspect_clicked()
{
  if (ui.touch->isChecked())
  {
    ui.touch->setChecked(false);
  }
}

void MainWindow::on_touch_clicked()
{
  if (ui.inspect->isChecked())
  {
    ui.inspect->setChecked(false);
  }
}

void MainWindow::on_btn_auto_manipulator_clicked()
{
  int mission_type, mission_shape;
  if (!ui.linear->isChecked() && !ui.omni->isChecked())
  {
    return;
  }
  if (!ui.inspect->isChecked() && !ui.touch->isChecked())
  {
    return;
  }
  if (ui.linear->isChecked())
  {
    mission_shape = 1;
  }
  if (ui.omni->isChecked())
  {
    mission_shape = 2;
  }
  if (ui.inspect->isChecked())
  {
    mission_type = 1;
  }
  if (ui.touch->isChecked())
  {
    mission_type = 2;
  }
  int mission_data = mission_shape * 10 + mission_type;

  ui.btn_auto_manipulator->setStyleSheet("QPushButton { color : green; }");
  ui.btn_auto_manipulator->setEnabled(false);
  ui.btn_victim_box->setEnabled(false);

  qnode.publishManipulatorMission(mission_data);
}

void MainWindow::slotUpdateCircuit()
{
  ui.battery_1->setText(QString::number(qnode.battery[0]));
  ui.battery_2->setText(QString::number(qnode.battery[1]));
  if (qnode.battery[0] >= 24.0)
  {
    ui.battery_1->setStyleSheet("QLabel { font: 18pt; color: green; }");
  }
  else if (qnode.battery[0] < 24.0 && qnode.battery[0] >= 23.2)
  {
    ui.battery_1->setStyleSheet("QLabel { font: 18pt; color: orange; }");
  }
  else
  {
    ui.battery_1->setStyleSheet("QLabel { font: 18pt; color: red; }");
  }
  if (qnode.battery[1] >= 24.0)
  {
    ui.battery_2->setStyleSheet("QLabel { font: 18pt; color: green; }");
  }
  else if (qnode.battery[1] < 24.0 && qnode.battery[1] >= 23.2)
  {
    ui.battery_2->setStyleSheet("QLabel { font: 18pt; color: orange; }");
  }
  else
  {
    ui.battery_2->setStyleSheet("QLabel { font: 18pt; color: red; }");
  }

  ui.btn_circuit->setStyleSheet("QPushButton { color : green; }");
  ui.btn_circuit->setEnabled(false);
}

void MainWindow::on_btn_manipulator_clicked()
{
  std::string target = "robot";
  std::string request = MANIPULATOR_REQUEST;
  std::string type = "shell";
  qnode.publishRequest(target, request, type);
}

void MainWindow::on_btn_slam_clicked()
{
  std::string target = "velodyne";
  std::string request = SLAM_REQUEST;
  std::string type = "shell";
  qnode.publishRequest(target, request, type);
}

void MainWindow::on_btn_circuit_clicked()
{
  std::string target = "robot";
  std::string request = CIRCUIT_REQUEST;
  std::string type = "shell";
  qnode.publishRequest(target, request, type);
}

void MainWindow::slotManipulatorUpdate()
{
  ui.motor_1->setText(QString::number(qnode.manipulator_angle[0]));
  ui.motor_2->setText(QString::number(qnode.manipulator_angle[1]));
  ui.motor_3->setText(QString::number(qnode.manipulator_angle[2]));
  ui.motor_4->setText(QString::number(qnode.manipulator_angle[3]));
  ui.motor_5->setText(QString::number(qnode.manipulator_angle[4]));
  ui.motor_6->setText(QString::number(qnode.manipulator_angle[5]));
  ui.motor_7->setText(QString::number(qnode.manipulator_angle[6]));
  ui.motor_8->setText(QString::number(qnode.manipulator_angle[7]));
}

void MainWindow::slotStatusUpdate(bool status)
{
  if (status)
  {
    ui.isconnected->setText("true");
    ui.isconnected->setStyleSheet("QLabel { color : green; }");
  }
  else if (!status)
  {
    ui.isconnected->setText("false");
    ui.isconnected->setStyleSheet("QLabel { color : red; }");
  }
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
    case 3:
      targetLabel = ui.img4;
      targetFps = ui.fps_4;
      targetSize = ui.img_size_4;
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
  ui.topic_img4->clear();
  ui.topic_img1->addItems(qnode.topicList);
  ui.topic_img2->addItems(qnode.topicList);
  ui.topic_img3->addItems(qnode.topicList);
  ui.topic_img4->addItems(qnode.topicList);
}

void MainWindow::slotUpdateState()
{
  ui.spd_L->setText(QString::number(qnode.rpm[0]));
  ui.spd_R->setText(QString::number(qnode.rpm[1]));
  setFlipperValueAndColor(ui.flipper_fl, qnode.flipper[0], qnode.flipper_status[0]);
  setFlipperValueAndColor(ui.flipper_rl, qnode.flipper[1], qnode.flipper_status[1]);
  setFlipperValueAndColor(ui.flipper_fr, qnode.flipper[2], qnode.flipper_status[2]);
  setFlipperValueAndColor(ui.flipper_rr, qnode.flipper[3], qnode.flipper_status[3]);
}

void MainWindow::slotUpdateJoyMode(int mode)
{
  if (mode == 0)
  {
    ui.joy_mode->setText("NON INIT JOY");
  }
  else if (mode == 1)
  {
    ui.joy_mode->setText("BASE JOY");
  }
  else if (mode == 2)
  {
    ui.joy_mode->setText("MANIPULATOR");
  }
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

void MainWindow::on_topic_img4_currentIndexChanged(int index)
{
  if (init)
  {
    QString topic = ui.topic_img4->currentText();
    qnode.img_topic[3] = topic.toStdString();
    qnode.changeTopic(3);
    std::cout << "[robot_operator] Changed cam 4 topic to : " << topic.toStdString() << std::endl;
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

void MainWindow::on_btn_send_clicked()
{
  std::string target = ui.target->currentText().toStdString();
  std::string request = ui.pub_log->toPlainText().toStdString();
  std::string type;
  if (ui.terminalSel->isChecked())
  {
    type = "shell";
  }
  else if (!ui.terminalSel->isChecked())
  {
    type = "NONE";
  }
  qnode.publishRequest(target, request, type);
}

void MainWindow::setFlipperValueAndColor(QLabel* label, double value, bool status)
{
  label->setText(QString::number(value));
  if (status)
  {
    label->setStyleSheet("QLabel { color: green; }");
  }
  else
  {
    label->setStyleSheet("QLabel { color: red; }");
  }
}

}  // namespace robot_operator
