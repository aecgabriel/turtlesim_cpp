#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <turtlesim/Pose.h>


class Go
{
    private:
        /* data */
        ros::Subscriber _subGoal;
        ros::Subscriber _subPose;
        ros::Publisher _pubCmdvel;
        ros::NodeHandle& _Node; 
        double _tolerance;
        double _poseTarget[2];
        double _pose[3]; // Pose with X, Y and Theta
        const float pi = 3.141592;
        void subPoseCallback(const turtlesim::Pose::ConstPtr&);
        void subGoalCallback_point(const geometry_msgs::Point::ConstPtr&);


    public:
        Go(ros::NodeHandle&, double p_tolerance);
        void moveto();

};