/**
 * @file /include/hitgroundcontrol/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef hitgroundcontrol_QNODE_HPP_
#define hitgroundcontrol_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QUdpSocket>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hitgroundcontrol {

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread {
  Q_OBJECT
public:
  QNode(int argc, char** argv, uint16_t minPort);
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();
  void launchCmd(char* cmd);


Q_SIGNALS:
  void rosShutdown();
  void showLog(char *type, QString logMsg);

private:
  int init_argc;
  char** init_argv;
  int minPort;
  QMap<QString, QUdpSocket*> udpMap;
  QList<QString> launchSteps;

private slots:
  void onVinsReadyRead();
  void onPx4ctrlReadyRead();
  void onTakeoffReadyRead();
  void onPlannerReadyRead();
};
}  // namespace hitgroundcontrol

#endif /* hitgroundcontrol_QNODE_HPP_ */
