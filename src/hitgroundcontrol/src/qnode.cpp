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

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
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
void QNode::launchCmd(char* cmd) {
  QString target_ip("127.0.0.1");
  QHostAddress target_addr(target_ip);
  // ROS_INFO("开始传输%s\n", cmd);
  udpMap[cmd]->writeDatagram(cmd, target_addr, 9999);
  // ROS_INFO("传输结束%s\n", cmd);
}
void QNode::onVinsReadyRead() {
  QByteArray datagram;
  datagram.resize(udpMap["vins"]->pendingDatagramSize());
  udpMap["vins"]->readDatagram(datagram.data(), datagram.size());
  QString str = datagram.data();
  // ROS_INFO("vins日志返回:%s\n", str.toStdString());
  Q_EMIT showLog("vins", str);
}
void QNode::onPx4ctrlReadyRead() {
  QByteArray datagram;
  datagram.resize(udpMap["px4ctrl"]->pendingDatagramSize());
  udpMap["px4ctrl"]->readDatagram(datagram.data(), datagram.size());
  QString str = datagram.data();
  Q_EMIT showLog("px4ctrl", str);
}
void QNode::onTakeoffReadyRead() {
  QByteArray datagram;
  datagram.resize(udpMap["takeoff"]->pendingDatagramSize());
  udpMap["takeoff"]->readDatagram(datagram.data(), datagram.size());
  QString str = datagram.data();
  Q_EMIT showLog("takeoff", str);
}
void QNode::onPlannerReadyRead() {
  QByteArray datagram;
  datagram.resize(udpMap["planner"]->pendingDatagramSize());
  udpMap["planner"]->readDatagram(datagram.data(), datagram.size());
  QString str = datagram.data();
  Q_EMIT showLog("planner", str);
}
}  // namespace hitgroundcontrol
