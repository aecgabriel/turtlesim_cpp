#include "goto.hpp"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

Go::Go(ros::NodeHandle& node, double tolerance): _Node(node), _tolerance(tolerance){

    _subPose = _Node.subscribe("/turtle1/pose",1, &Go::subPoseCallback, this);
    _subGoal = _Node.subscribe("/turtle1/goal", 1, &Go::subGoalCallback_point, this);
    _pubCmdvel = _Node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    _poseTarget[0] = 5.544444561;
    _poseTarget[1] = 5.544444561;
    
}

void Go::subPoseCallback(const turtlesim::Pose::ConstPtr& msg) {

    _pose[0] = msg->x;
    _pose[1] = msg->y;
    _pose[2] = msg->theta;
}
void Go::subGoalCallback_point(const geometry_msgs::Point::ConstPtr& msg) {

    _poseTarget[0] = msg->x;
    _poseTarget[1] = msg->y;  
    ROS_INFO("%f %f", _poseTarget[0], _poseTarget[1]);
}   

void Go::moveto(){
    geometry_msgs::Twist speed;
    double linearVelocity;
    double angle =  atan2(_poseTarget[1] - _pose[1], _poseTarget[0] - _pose[0]);
    double error_angle = angle - _pose[2];
    double target_distance = sqrt(pow(_poseTarget[0] - _pose[0],2) + pow(_poseTarget[1] - _pose[1],2));
    ROS_INFO("Erro: %f", target_distance);
    if(error_angle > pi){
        error_angle -= 2*(pi);
        }
    else if (error_angle < -pi){
        error_angle += 2*(pi);
        }

    speed.angular.z = error_angle*.5;
    if (speed.angular.z > .5){
        speed.angular.z = .5;
    }
    else if (speed.angular.z < -.5){
        speed.angular.z = -.5;
    }
    if (target_distance > _tolerance){
        speed.linear.x = fabs(target_distance)*.5;
        if (speed.linear.x > 1){
            speed.linear.x = 1;
            }
    }
    else{
        speed.linear.x = speed.angular.z = 0;
    }
    
    _pubCmdvel.publish(speed);
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "turtle_bot_move");
    ros::NodeHandle node;
    Go  Duck_bot_go(node, 0.1);
    while (ros::ok())
    {
        Duck_bot_go.moveto();
        ros::spinOnce();
    }
    ROS_WARN("ACABOU");
}