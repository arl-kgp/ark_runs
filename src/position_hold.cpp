#include <ros/ros.h>
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "ark_msgs/PidErrors.h"
#include <nav_msgs/Odometry.h>


class PosHold
{
    ros::NodeHandle nh_;
    ros::Publisher pid_pub;
    ros::Subscriber odom_sub;

public:
    PosHold()
    {
        pid_pub = nh_.advertise<ark_msgs::PidErrors>("/ark/pid_errors", 1000);
        odom_sub = nh_.subscribe("/odometry/filtered", 10, &PosHold::odomCb, this);
        target_x = 0; target_y = 0; target_z = 0;      
    }

    ~PosHold()
    {
    } 

    void set_target(float in_x, float in_y, float in_z)
    {
        target_x = in_x;
        target_y = in_y;
        target_z = in_z;
    }
  
private:
    float target_z;
    float target_x;
    float target_y;

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        float diff_x = msg->pose.pose.position.x - target_x;
        float diff_y = msg->pose.pose.position.y - target_y;
        float diff_z = msg->pose.pose.position.z - target_z;

        ark_msgs::PidErrors pid_error;
        pid_error.dx = diff_x;
        pid_error.dy = 0;
        pid_error.dz = 0;
        pid_pub.publish(pid_error);

        ROS_INFO("%.2f, %.2f, %.2f", diff_x, diff_y, diff_z);
    }   
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pos_hold");
    ros::NodeHandle n;

    ros::ServiceClient set_mode_cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.base_mode = 0;
    set_mode_srv.request.custom_mode = "ALT_HOLD";
    if (set_mode_cl.call(set_mode_srv))
    {
        ROS_INFO("TRUE");
    }
    else
    {
        ROS_ERROR("FALSE");
    }

    ros::ServiceClient set_arm_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool set_arm;
    set_arm.request.value = true;
    if (set_arm_cl.call(set_arm))
    {
        ROS_INFO("TRUE");
    }
    else
    {
        ROS_ERROR("FALSE");
    }

    PosHold ah;
    ah.set_target(-1.9, 0.0, 0.0);
    ros::spin();
    return 0;
}