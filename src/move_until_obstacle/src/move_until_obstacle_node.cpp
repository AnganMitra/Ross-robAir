// move until obstacle using lidar data

#include "ros/ros.h"
#include "ros/time.h"
// We use the already built messages types
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#define detection_range 0.4

using namespace std;

class move_until_obstacle {

private:
    ros::NodeHandle n;
    ros::Subscriber sub_scan;
    ros::Publisher pub_cmd_vel;
    ros::Publisher pub_obstacle_float;
	float obstacle_float;
public:

move_until_obstacle() {

    // receive data of laserscanner
    sub_scan = n.subscribe("scan", 1, &move_until_obstacle::scanCallback, this);

    // send command to cmd_vel
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	//communication with transrot
	pub_obstacle_float = n.advertise<std_msgs::Float32>("obstacle_float", 0);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
//the robot moves in translation at 1meter per second and stops if it perceives an obstacle at less than 1meter in angle between -20degrees and +20degrees
    

    int obstacle = 0;
    
    int loop = 342;//starting angle at about -20 degrees

    float beam_angle = scan->angle_min + scan->angle_increment*loop;

    while ( ( loop <= 465 ) && not ( obstacle ) ) {//we stop if there an obstacle at less than 1 meter between -20 degres and +20degres
        obstacle = ( scan->ranges[loop] < detection_range ) && ( scan->ranges[loop] > scan->range_min);
        if ( obstacle )
            ROS_INFO("obstacle detected at %f where the distance is %f", beam_angle*180/M_PI, scan->ranges[loop]);
        loop++;
        beam_angle += scan->angle_increment;
    }
    
    //we first perform the rotation_to_do and secondly the translation_to_do
    std_msgs::Float32 msg_obstacle_float;
	if (obstacle)
		obstacle_float = 1.0;
	else
		obstacle_float = 0.0;
		
    //to complete
    msg_obstacle_float.data = obstacle_float;
    pub_obstacle_float.publish(msg_obstacle_float);
	/*
    geometry_msgs::Twist twist;

    if ( obstacle ) {

        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;

        pub_cmd_vel.publish(twist);
     }
	*/
	
}

};


    int main(int argc, char **argv){

        ros::init(argc, argv, "move_until_obstacle");

        move_until_obstacle bsObject;

        ros::spin();

        return 0;
    }


