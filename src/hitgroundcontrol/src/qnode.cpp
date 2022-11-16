/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/hitgroundcontrol/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hitgroundcontrol {

/*****************************************************************************
** Implementation
*****************************************************************************/
QNode::QNode(int argc, char** argv, uint16_t port):
        init_argc(argc),
        init_argv(argv),
        minPort(port)
{
  launchSteps = {"vins", "px4ctrl", "takeoff", "planner"};
  for (int i = 0; i < launchSteps.length(); i++) {
    QUdpSocket *socket = new QUdpSocket(this);
    socket->bind(minPort + i);
    udpMap[launchSteps[i]] = socket;
  }
  connect(udpMap[launchSteps[0]], SIGNAL(readyRead()), this, SLOT(onVinsReadyRead()));
  connect(udpMap[launchSteps[1]], SIGNAL(readyRead()), this, SLOT(onPx4ctrlReadyRead()));
  connect(udpMap[launchSteps[2]], SIGNAL(readyRead()), this, SLOT(onTakeoffReadyRead()));
  connect(udpMap[launchSteps[3]], SIGNAL(readyRead()), this, SLOT(onPlannerReadyRead()));
}
QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"hitgroundcontrol");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.
  start();
  return true;
}

bool QNode::init(const std::string &master_url) {
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  ros::init(remappings,"hitgroundcontrol");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.
  start();
  return true;
}

void QNode::run() {
  ros::Rate loop_rate(1);
  while ( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
void QNode::launchCmd(char *cmd) {
  QString target_ip("127.0.0.1");
  QHostAddress target_addr(target_ip);
  udpMap[cmd]->writeDatagram(cmd, target_addr, 9999);
}
void QNode::onVinsReadyRead() {
  char buf[512];
  qint64 len = udpMap["vins"]->pendingDatagramSize();
  udpMap["vins"]->readDatagram(buf, len);
  Q_EMIT showLog("vins", buf);
}
void QNode::onPx4ctrlReadyRead() {
  char buf[512];
  qint64 len = udpMap["px4ctrl"]->pendingDatagramSize();
  udpMap["px4ctrl"]->readDatagram(buf, len);
  Q_EMIT showLog("px4ctrl", buf);
}
void QNode::onTakeoffReadyRead() {
  char buf[512];
  qint64 len = udpMap["takeoff"]->pendingDatagramSize();
  udpMap["takeoff"]->readDatagram(buf, len);
  Q_EMIT showLog("takeoff", buf);
}
void QNode::onPlannerReadyRead() {
  char buf[512];
  qint64 len = udpMap["planner"]->pendingDatagramSize();
  udpMap["planner"]->readDatagram(buf, len);
  Q_EMIT showLog("planner", buf);
}
}  // namespace hitgroundcontrol
