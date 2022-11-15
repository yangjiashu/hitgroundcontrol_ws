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
#include "../include/hitgroundcontrol/main_window.hpp"
#include "../include/hitgroundcontrol/network_utils.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hitgroundcontrol {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
        : argc(argc), argv(argv), QMainWindow(parent)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  if ( ui.checkbox_remember_settings->isChecked() ) {
    on_button_connect_clicked(true);
  }
}

MainWindow::~MainWindow() {}

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
  QNode *node = new QNode(argc, argv, 60010);
  nodeList.append(node);
  QObject::connect(node, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(node, SIGNAL(showLog(char*, QString)), this, SLOT(on_show_log_emit(char*, QString)));
  if ( ui.checkbox_use_environment->isChecked() ) {
    if ( !node->init() ) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
    }
  } else {
    if ( !node->init(ui.line_edit_master->text().toStdString(),
                      ui.line_edit_host->text().toStdString()) ) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
      ui.line_edit_master->setReadOnly(true);
      ui.line_edit_host->setReadOnly(true);
      ui.line_edit_topic->setReadOnly(true);
    }
  }
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
  bool enabled;
  if ( state == 0 ) {
    enabled = true;
  } else {
    enabled = false;
  }
  ui.line_edit_master->setEnabled(enabled);
  ui.line_edit_host->setEnabled(enabled);
}

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "hitgroundcontrol");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
  QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
  //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
  ui.line_edit_master->setText(master_url);
  ui.line_edit_host->setText(host_url);
  //ui.line_edit_topic->setText(topic_name);
  bool remember = settings.value("remember_settings", false).toBool();
  ui.checkbox_remember_settings->setChecked(remember);
  bool checked = settings.value("use_environment_variables", false).toBool();
  ui.checkbox_use_environment->setChecked(checked);
  if ( checked ) {
    ui.line_edit_master->setEnabled(false);
    ui.line_edit_host->setEnabled(false);
  }
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "hitgroundcontrol");
  settings.setValue("master_url",ui.line_edit_master->text());
  settings.setValue("host_url",ui.line_edit_host->text());
  //settings.setValue("topic_name",ui.line_edit_topic->text());
  settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
  QMainWindow::closeEvent(event);
}

void MainWindow::on_show_log_emit(char* type, QString logMsg) {
  // TODO: 这个地方有问题 type参数
  printf("%s\n", type);
  ui.txtEdit_launchVins->appendPlainText(logMsg);
}
void MainWindow::on_btn_launchVins_clicked() {
  nodeList[0]->launchCmd("vins");
}
void MainWindow::on_btn_launchPx4ctrl_clicked() {
  nodeList[0]->launchCmd("px4ctrl");
}
void MainWindow::on_btn_launchTakeoff_clicked() {
  nodeList[0]->launchCmd("takeoff");
}
void MainWindow::on_btn_launchPlanner_clicked() {
  nodeList[0]->launchCmd("planner");
}
}  // namespace hitgroundcontrol

