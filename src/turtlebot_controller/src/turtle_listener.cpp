#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"

ros::Publisher pub;

void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
{
	ROS_INFO("Turtle subscriber@[%f, %f, %f]",
	msg->x, msg->y, msg->theta);
    	geometry_msgs::Twist my_vel;
    	my_vel.linear.x = 1.0;    // Move forward at 1 m/s
    	my_vel.angular.z = 1.0;   // Rotate at 1 rad/s
    // Publish the velocity command
   	pub.publish(my_vel);
}


int main (int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS
	//system
	ros::init(argc, argv, "turtlebot_subscriber");
	ros::NodeHandle nh;

	ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("/spawn");
    // Prepare the spawn request
    	turtlesim::Spawn spawnSrv;
    	spawnSrv.request.x = 5.0;
    	spawnSrv.request.y = 5.0;
    	spawnSrv.request.theta = 0.0;
    	spawnSrv.request.name = "Turtle_Nurberdi";

    // Call the spawn service
    	if (spawnClient.call(spawnSrv)) {
        	ROS_INFO("Spawned a new turtle named: %s", spawnSrv.response.name.c_str());
    	} else {
        	ROS_ERROR("Failed to call service /spawn");
    	}

	// Define the subscriber to turtle's position
	ros::Subscriber sub = nh.subscribe("turtle1/pose", 1,turtleCallback);

    	// Define the publisher to control the turtle
    	pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

	ros::spin();
	return 0;
}
