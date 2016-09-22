#include <ros/ros.h>
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "ark_msgs/PidErrors.h"
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Geometry>


class PosHold {
    ros::NodeHandle nh_;
    ros::Publisher pid_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber setpoint_sub;

 public:
    PosHold() {
        pid_pub = nh_.advertise<ark_msgs::PidErrors>("/ark/pid_errors", 1000);
        odom_sub = nh_.subscribe("/odometry/filtered", 10, &PosHold::odomCb, this);
        setpoint_sub = nh_.subscribe("/ark/setpoint", 10, &PosHold::setpointCb, this);
        target_x = 0; target_y = 0; target_z = 0; target_psi = 0;
        initial_psi = 0;
        ran_before  = false;   
    }

    ~PosHold()
    {
    } 

    void set_target(float in_x, float in_y, float in_z, float in_psi)
    {
        target_x = in_x;
        target_y = in_y;
        target_z = in_z;
        target_psi = in_psi;
    }
  
private:
    float target_z;
    float target_x;
    float target_y;
    float target_psi;
    bool ran_before;
    float initial_psi;

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

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        float diff_x = msg->pose.pose.position.x - target_x;
        float diff_y = msg->pose.pose.position.y - target_y;
        float diff_z = msg->pose.pose.position.z - target_z;
        Quaternionm myq;
        double yaw, pitch, roll;
        myq.x = msg->pose.pose.orientation.x;
        myq.y = msg->pose.pose.orientation.y;
        myq.z = msg->pose.pose.orientation.z;
        myq.w = msg->pose.pose.orientation.w;
        GetEulerAngles(myq, yaw, pitch, roll);
        float diff_psi = yaw - (initial_psi + target_psi);

        ark_msgs::PidErrors pid_error;
        pid_error.dx = diff_x;
        pid_error.dy = diff_y;
        pid_error.dz = diff_z;
        pid_error.dpsi = diff_psi;
        pid_error.vx = - msg->twist.twist.linear.x;
        pid_error.vy = - msg->twist.twist.linear.y;
        pid_error.vz = - msg->twist.twist.linear.z;

        if (!ran_before)
        {
            ran_before = true;
            initial_psi = yaw;
            return;
        }

        pid_pub.publish(pid_error);

        ROS_INFO("%.2f, %.2f, %.2f, %.2f", diff_x, diff_y, diff_z, diff_psi);
    }

    void setpointCb(const ark_msgs::PidErrorsConstPtr& msg)
    {
        target_x = msg->dx;
        target_y = msg->dy;
        target_z = msg->dz;
        target_psi = msg->dpsi;
        ran_before = false;
        ROS_INFO("Setpoint changed to %.2f, %.2f, %.2f, %.2f", target_x, target_y, target_z, target_psi);
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
    ah.set_target(0.0, 0.0, 2, 0.0);
    ros::spin();
    return 0;
}