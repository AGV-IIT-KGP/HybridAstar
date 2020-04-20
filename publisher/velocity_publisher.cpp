#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h" 
#include "geometry_msgs/TransformStamped.h" 
#include "sensor_msgs/Range.h"
#include <iostream>
#include "../include/State.hpp"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "tf2/convert.h" 
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std;

State curr, goal;
bool dest_ch = false;
int distThreshold = 5;
tf2_ros::Buffer tfBuffer;
bool is_left_safe = true;
bool is_right_safe = true;
// Used to locate the current position of vehicle
void odomCallback(const nav_msgs::Odometry& odom_msg) 
{
    // cout<<"Inside OdomCallback"<<endl;

    curr.x = odom_msg.pose.pose.position.x ;
    curr.y = odom_msg.pose.pose.position.y ;

    tf::Quaternion q(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    curr.theta = fmod(yaw+2*M_PI,2*M_PI);
    cout<<"Starting Received    : "<<curr.x<<" "<<curr.y<<" "<<curr.theta<<endl;
    cout<<"Destination Received : "<<goal.x<<" "<<goal.y<<" "<<goal.theta<<endl;

}
void left_sonar_call_back (sensor_msgs::Range l_dist)
{
    float left = l_dist.range;
    cout<<"left  "<<left<<endl;
    if(left<1.8)
        is_left_safe = false;
    else
        is_left_safe = true;
}
void right_sonar_call_back (sensor_msgs::Range r_dist)
{
    float right = r_dist.range;
    cout<<"right "<<right<<endl;
    if(right<1.8)
        is_right_safe = false;
        else is_right_safe = true;
    
}

void goalCallback(const geometry_msgs::PoseStamped&  target)
{
    // cout<<"Inside goalCallback"<<endl;
    dest_ch = true;
    
    geometry_msgs::PoseStamped  trans_goal;
    geometry_msgs::TransformStamped trans_msg;
    
    try{
        trans_msg = tfBuffer.lookupTransform("map", target.header.frame_id, ros::Time(0));
        tf2::doTransform(target,trans_goal,trans_msg);
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }
    
    goal.x= trans_goal.pose.position.x ;
    goal.y= trans_goal.pose.position.y ;
        
    tf::Quaternion q(trans_goal.pose.orientation.x, trans_goal.pose.orientation.y, trans_goal.pose.orientation.z, trans_goal.pose.orientation.w);
    tf::Matrix3x3 m(q);
    
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    goal.theta=fmod(yaw+2*M_PI,2*M_PI);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_frenet", 10);
    ros::Subscriber odom  = nh.subscribe("/base_pose_ground_truth",10,&odomCallback);
    ros::Subscriber destination  = nh.subscribe("/hybrid_astar_goal",10,&goalCallback);
    ros::Subscriber lsub = nh.subscribe("/prius/front_sonar/left_far_range",1,left_sonar_call_back);
    ros::Subscriber rsub= nh.subscribe("/prius/front_sonar/right_far_range",1,right_sonar_call_back);
    tf2_ros::TransformListener listener(tfBuffer);
    int check;
    while(!dest_ch)
    {
        cout<<"Waiting for Goal "<<endl;
        ros::spinOnce();
    }

    ros::Rate rate(10);
    while (ros::ok())
    {
        geometry_msgs::Twist vel;
        if( sqrt((curr.x-goal.x)*(curr.x-goal.x) + (curr.y-goal.y)*(curr.y-goal.y))< distThreshold)
            vel.linear.x = 0;
        else if(is_right_safe&&is_left_safe)
            vel.linear.x = 1.5;
        else if(is_right_safe == false && is_left_safe == false)
            vel.linear.x=0;
        else
            vel.linear.x = 0.75;
        nh.getParam("is_safe",check);
        if(check==0)
            vel.linear.x=0;
        pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

