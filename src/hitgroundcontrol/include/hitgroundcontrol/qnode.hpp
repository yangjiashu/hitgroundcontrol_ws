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
#include <QStringListModel>


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
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();

Q_SIGNALS:
  void rosShutdown();

private:
  int init_argc;
  char** init_argv;
};

}  // namespace hitgroundcontrol

#endif /* hitgroundcontrol_QNODE_HPP_ */
