#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/String.h>
#include <string>

int main(int argc, char **argv)
{
    // TODO: argc必须大于等于3 即必须传入两个参数，且第一个符号不能为数字
    std::string node_name;
    node_name = argv[1] + std::string("_") + argv[2] + std::string("_communication");
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    

    return 0;
}

class Communicator
{
private:
    std::string vehicle_type;
    int vehicle_id;
    geometry_msgs::PoseStamped currentPose = geometry_msgs::PoseStamped();
    int hover_yaw = 0;
    int hover_flag = 0;
    mavros_msgs::PositionTarget target_motion = mavros_msgs::PositionTarget();
    bool arm_state = false;
    int motion_type = 0;
    std::string flight_mode;
    std::string mission;
    std::string last_cmd;
    int hold_position_x = 0;
    int hold_position_y = 0;
    int hold_position_z = 0;
    int hold_yaw = 0;
    int hold_flag = 0;
    int hold_x_flag = 0;
    int hold_y_flag = 0;
    int hold_z_flag = 0;
    int hold_yaw_flag = 0;
    int hold_kp_x = 1;
    int hold_kp_y = 1;
    int hold_kp_z = 1;
    int hold_kp_yaw = 1;

public:
    Communicator(std::string vehicle_type, int vehicle_id);
    ~Communicator();
};

Communicator::Communicator(std::string vehicle_type, int vehicle_id)
{
    vehicle_type = vehicle_type;
    vehicle_id = vehicle_id;
}

Communicator::~Communicator()
{
}
