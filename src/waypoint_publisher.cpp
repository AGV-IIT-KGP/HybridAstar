#include "ros/ros.h"
#include "rosgraph_msgs/Log.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"
#include <string.h>
#include "geometry_msgs/PoseArray.h" 
#include "geometry_msgs/Pose.h" 

#include <sys/time.h>
#include <visualization_msgs/Marker.h> 
#include <tf/transform_listener.h>
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/core/core.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
using namespace std;
using namespace cv;
using namespace ros;


float curr_x,curr_y,curr_theta;
void odom_call_back(const nav_msgs::Odometry::ConstPtr& odom_msg) 
    {
 
        curr_x = odom_msg->pose.pose.position.x;
        curr_y = odom_msg->pose.pose.position.y;

        tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        curr_theta = fmod(yaw+2*M_PI,2*M_PI);

        
    }



int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_node");
	ros::NodeHandle nh;
    
	ros::Subscriber odom_call_sub = nh.subscribe("/base_pose_ground_truth", 1, &odom_call_back);
   FILE *ptr;
   ptr = fopen("src/hybrid_astar/waypoints.txt","r");

   int i,j;
   float x,y,theta;
   bool flag=false;
   geometry_msgs::PoseStamped waypoint_bot;
   waypoint_bot.header.frame_id = "map";
   waypoint_bot.header.stamp = ros::Time::now();
   Publisher waypoint_publisher = nh.advertise<geometry_msgs::PoseStamped>("/hybrid_astar_goal",1);
      
	ros::Rate wait_rate(1);


			fscanf(ptr,"%f%f%f",&x,&y,&theta);
	   		cout<<x<<" "<<y<<" "<<theta<<endl;

	   		waypoint_bot.pose.position.x = x;
			waypoint_bot.pose.position.y = y;
			waypoint_bot.pose.position.z = 0;
   			tf::Quaternion frame_qt = tf::createQuaternionFromYaw(theta);
			waypoint_bot.pose.orientation.x = frame_qt.x();
			waypoint_bot.pose.orientation.y = frame_qt.y();
			waypoint_bot.pose.orientation.z = frame_qt.z();
			waypoint_bot.pose.orientation.w = frame_qt.w();
			waypoint_publisher.publish(waypoint_bot);
      
      cout<<"Publishedddddddddddd"<<endl;
     

			
   while(!feof(ptr))
   {
   		if(flag)
   		{
	   		fscanf(ptr,"%f%f%f",&x,&y,&theta);
	   		cout<<x<<" "<<y<<" "<<theta<<endl;

	   		waypoint_bot.pose.position.x = x;
			waypoint_bot.pose.position.y = y;
			waypoint_bot.pose.position.z = 0;
   			frame_qt = tf::createQuaternionFromYaw(theta);
			waypoint_bot.pose.orientation.x = frame_qt.x();
			waypoint_bot.pose.orientation.y = frame_qt.y();
			waypoint_bot.pose.orientation.z = frame_qt.z();
			waypoint_bot.pose.orientation.w = frame_qt.w();
      flag = false;

   		}
   		cout<<sqrt((curr_x-x)*(curr_x-x)+(curr_y-y)*(curr_y-y))<<"  "<<x<<" "<<y<<endl;
   		if(sqrt((curr_x-x)*(curr_x-x)+(curr_y-y)*(curr_y-y))<3)
   		{
   			waypoint_publisher.publish(waypoint_bot);
   			flag=true;
   		}
   		waypoint_publisher.publish(waypoint_bot);
   		ros::spinOnce();
   		wait_rate.sleep();
   }


	return 0;
}
