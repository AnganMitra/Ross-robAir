// move until obstacle using lidar data

#include "ros/ros.h"
#include "ros/time.h"
// We use the already built messages types
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>

#define detection_range 0.5

using namespace std;

class move_until_obstacle {

private:
    ros::NodeHandle n;
    ros::Subscriber sub_scan;
    ros::Publisher pub_cmd_vel;

public:

move_until_obstacle() {

    // receive data of laserscanner
    sub_scan = n.subscribe("scan", 1, &move_until_obstacle::scanCallback, this);

    // send command to cmd_vel
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
//the robot moves in translation at 1meter per second and stops if it perceives an obstacle at less than 1meter in angle between -20degrees and +20degrees
    ;

    int obstacle = 0;
    int loop = 450;//starting angle at about -20 degrees

    float beam_angle = scan->angle_min + scan->angle_increment*loop;

    while ( ( loop <= 550 ) && not ( obstacle ) ) {//we stop if there an obstacle at less than 1 meter between -20 degres and +20degres
        obstacle = ( scan->ranges[loop] < detection_range ) && ( scan->ranges[loop] > scan->range_min);
        if ( obstacle )
            ROS_INFO("obstacle detected at %f where the distance is %f", beam_angle*180/M_PI, scan->ranges[loop]);
        loop++;
        beam_angle += scan->angle_increment;
    }

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


}

};


    int main(int argc, char **argv){

        ros::init(argc, argv, "move_until_obstacle");

        move_until_obstacle bsObject;

        ros::spin();

        return 0;
    }


