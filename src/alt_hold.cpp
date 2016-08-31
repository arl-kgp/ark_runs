#include <ros/ros.h>
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "ark_msgs/PidErrors.h"
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>


class AltHold
{
    ros::NodeHandle nh_;
    ros::Publisher pid_pub;
    ros::Subscriber alt_sub;

public:
    AltHold()
    {
        pid_pub = nh_.advertise<ark_msgs::PidErrors>("/ark/pid_errors", 1000);
        alt_sub = nh_.subscribe("/odometry/filtered", 1000, &AltHold::altCb, this);
        target_height = 0;
        target_yaw = 0;
        initial_yaw = 0;
        ran_before  = false;  
    }

    ~AltHold()
    {
    } 

    void set_target_height(float input)
    {
        target_height = input;
    }

    void set_target_yaw(float input)
    {
        target_yaw = input;
    }
  
private:
    float target_height;
    float target_yaw;
    bool ran_before;
    float initial_yaw;

    struct Quaternionm
    {
        double w, x, y, z;
    };

    void GetEulerAngles(Quaternionm q, double& yaw, double& pitch, double& roll)
    {
        const double w2 = q.w*q.w;
        const double x2 = q.x*q.x;
        const double y2 = q.y*q.y;
        const double z2 = q.z*q.z;
        const double unitLength = w2 + x2 + y2 + z2;    // Normalised == 1, otherwise correction divisor.
        const double abcd = q.w*q.x + q.y*q.z;
        const double eps = 1e-7;    // TODO: pick from your math lib instead of hardcoding.
        const double pi = 3.14159265358979323846;   // TODO: pick from your math lib instead of hardcoding.
        if (abcd > (0.5-eps)*unitLength)
        {
            yaw = 2 * atan2(q.y, q.w);
            pitch = pi;
            roll = 0;
        }
        else if (abcd < (-0.5+eps)*unitLength)
        {
            yaw = -2 * ::atan2(q.y, q.w);
            pitch = -pi;
            roll = 0;
        }
        else
        {
            const double adbc = q.w*q.z - q.x*q.y;
            const double acbd = q.w*q.y - q.x*q.z;
            yaw = ::atan2(2*adbc, 1 - 2*(z2+x2));
            pitch = ::asin(2*abcd/unitLength);
            roll = ::atan2(2*acbd, 1 - 2*(y2+x2));
        }
    }

    void altCb(const nav_msgs::OdometryConstPtr& msg)
    {
        float current_altitude = msg->pose.pose.position.z;
        float alt_diff = current_altitude - target_height;

        ark_msgs::PidErrors pid_error;
        Quaternionm myq;
        double yaw, pitch, roll;
        myq.x = msg->pose.pose.orientation.x;
        myq.y = msg->pose.pose.orientation.y;
        myq.z = msg->pose.pose.orientation.z;
        myq.w = msg->pose.pose.orientation.w;  
        GetEulerAngles(myq, yaw, pitch, roll);
        pid_error.dz = alt_diff;

        float yaw_diff = yaw - (initial_yaw + target_yaw);
        pid_error.dpsi = yaw_diff;

        if (!ran_before)
        {
            ran_before = true;
            initial_yaw = yaw;
            return;
        }

        pid_pub.publish(pid_error);

        ROS_INFO("%.2f , %.2f", alt_diff, yaw_diff);
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
    ah.set_target_height(2.0);
    ah.set_target_yaw(0.0);
    ros::spin();
    return 0;

    /*
    TODO: 
    mode ALT_HOLD
    Arm throttle
    Detect alt error and push errors
    */
}