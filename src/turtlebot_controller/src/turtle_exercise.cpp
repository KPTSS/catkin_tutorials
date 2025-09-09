#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <turtlesim/TeleportAbsolute.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_exercise_node");
    ros::NodeHandle nh;
    
    // Wait for services to be available
    ros::service::waitForService("/kill");
    ros::service::waitForService("/spawn");
    
    // 1. Kill the default turtle
    ros::ServiceClient killClient = nh.serviceClient<turtlesim::Kill>("/kill");
    turtlesim::Kill killSrv;
    killSrv.request.name = "turtle1";
    
    if (killClient.call(killSrv)) {
        ROS_INFO("Successfully killed turtle1");
    } else {
        ROS_ERROR("Failed to kill turtle1");
        return 1;
    }
    
    // 2. Spawn new turtle named turtle_Nurberdi at center
    ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawnSrv;
    spawnSrv.request.x = 5.5;
    spawnSrv.request.y = 5.5;
    spawnSrv.request.theta = 0.0;
    spawnSrv.request.name = "turtle_Nurberdi";
    
    if (spawnClient.call(spawnSrv)) {
        ROS_INFO("Successfully spawned turtle_Nurberdi");
    } else {
        ROS_ERROR("Failed to spawn turtle_Nurberdi");
        return 1;
    }
    
    // 3. Teleport to bottom-left corner (1,1)
    ros::ServiceClient teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle_Nurberdi/teleport_absolute");
    turtlesim::TeleportAbsolute teleportSrv;
    teleportSrv.request.x = 1.0;
    teleportSrv.request.y = 1.0;
    teleportSrv.request.theta = 0.0;
    
    if (teleportClient.call(teleportSrv)) {
        ROS_INFO("Teleported turtle_Nurberdi to corner (1,1)");
    } else {
        ROS_ERROR("Failed to teleport turtle");
        return 1;
    }
    
    // 4. Create publisher for velocity commands
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle_Nurberdi/cmd_vel", 10);
    ros::Duration(1.0).sleep(); // Wait for publisher to connect
    
    // SQUARE TRAJECTORY: Move from corner to corner
    ROS_INFO("Starting square trajectory...");
    
    // Square corners: (1,1) → (1,10) → (10,10) → (10,1) → (1,1)
    float square_corners[4][2] = {{1,10}, {10,10}, {10,1}, {1,1}};
    
    for(int corner = 0; corner < 4; corner++) {
        // Calculate distance and angle to next corner
        float target_x = square_corners[corner][0];
        float target_y = square_corners[corner][1];
        
        // Move to the corner
        teleportSrv.request.x = target_x;
        teleportSrv.request.y = target_y;
        teleportSrv.request.theta = 0.0;
        teleportClient.call(teleportSrv);
        
        ROS_INFO("Moved to corner (%f, %f)", target_x, target_y);
        ros::Duration(1.0).sleep(); // Pause at each corner
    }
    
    // TRIANGULAR TRAJECTORY: Move between three corners with diagonal return
    ROS_INFO("Starting triangular trajectory...");
    
    // Triangle corners: (1,1) → (1,10) → (10,10) → diagonal back to (1,1)
    float triangle_corners[3][2] = {{1,10}, {10,10}, {1,1}};
    
    // Move to first triangle corner (1,10)
    teleportSrv.request.x = triangle_corners[0][0];
    teleportSrv.request.y = triangle_corners[0][1];
    teleportSrv.request.theta = 0.0;
    teleportClient.call(teleportSrv);
    ROS_INFO("Moved to triangle corner 1: (%f, %f)", triangle_corners[0][0], triangle_corners[0][1]);
    ros::Duration(1.0).sleep();
    
    // Move to second triangle corner (10,10)
    teleportSrv.request.x = triangle_corners[1][0];
    teleportSrv.request.y = triangle_corners[1][1];
    teleportSrv.request.theta = 0.0;
    teleportClient.call(teleportSrv);
    ROS_INFO("Moved to triangle corner 2: (%f, %f)", triangle_corners[1][0], triangle_corners[1][1]);
    ros::Duration(1.0).sleep();
    
    // Move diagonally back to start (1,1) - using teleport for diagonal movement
    teleportSrv.request.x = triangle_corners[2][0];
    teleportSrv.request.y = triangle_corners[2][1];
    teleportSrv.request.theta = 0.0;
    teleportClient.call(teleportSrv);
    ROS_INFO("Moved diagonally back to start: (%f, %f)", triangle_corners[2][0], triangle_corners[2][1]);
    
    ROS_INFO("Exercise completed successfully! Both trajectories executed.");
    return 0;
}
