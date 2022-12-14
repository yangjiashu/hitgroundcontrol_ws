/**
 * @file /include/hitgroundcontrol/main_window.hpp
 *
 * @brief Qt based gui for hitgroundcontrol.
 *
 * @date November 2010
 **/
#ifndef hitgroundcontrol_MAIN_WINDOW_H
#define hitgroundcontrol_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QUdpSocket>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace hitgroundcontrol
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
  MainWindow(int argc, char **argv, QWidget *parent = 0);
  ~MainWindow();

  void ReadSettings();  // Load up qt program settings at startup
  void WriteSettings(); // Save qt program settings when closing

  void closeEvent(QCloseEvent *event); // Overloaded function
  void showNoMasterMessage();

public Q_SLOTS:
  /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
  void on_button_connect_clicked(bool check);
  void on_checkbox_use_environment_stateChanged(int state);


private slots:
  void on_show_log_emit(char *type, char *logMsg);
  void on_btn_launchVins_clicked();
  void on_btn_launchTakeoff_clicked();
  void on_btn_launchPlanner_clicked();
  void on_btn_launchPx4ctrl_clicked();

private:
  int argc;
  char **argv;
  Ui::MainWindowDesign ui;
  QList<QNode*> nodeList;
};

} // namespace hitgroundcontrol

#endif // hitgroundcontrol_MAIN_WINDOW_H
