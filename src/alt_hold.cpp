#include <ros/ros.h>
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "ark_msgs/PidErrors.h"
#include <std_msgs/Float64.h>


class AltHold
{
    ros::NodeHandle nh_;
    ros::Publisher pid_pub;
    ros::Subscriber alt_sub;

public:
    AltHold()
    {
        pid_pub = nh_.advertise<ark_msgs::PidErrors>("/ark/pid_errors", 1000);
        alt_sub = nh_.subscribe("/mavros/global_position/rel_alt", 1000, &AltHold::altCb, this);
        target_height = 0;      
    }

    ~AltHold()
    {
    } 

    void set_target_height(float input)
    {
        target_height = input;
    }
  
private:
    float target_height;

    void altCb(const std_msgs::Float64::ConstPtr& msg)
    {
        float current_altitude = msg->data;
        float diff = current_altitude - target_height;

        ark_msgs::PidErrors pid_error;
        pid_error.dz = diff;
        pid_pub.publish(pid_error);

        ROS_INFO("%.2f", diff);
    }   
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "alt_hold");
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

    AltHold ah;
    ah.set_target_height(5.0);
    ros::spin();
    return 0;

    /*
    TODO: 
    mode ALT_HOLD
    Arm throttle
    Detect alt error and push errors
    */
}