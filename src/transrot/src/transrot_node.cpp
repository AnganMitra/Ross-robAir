#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#define translation_threshold 0.5
#define rotation_threshold 0.5
class transrot {
private:

    ros::NodeHandle n;
    // communication with cmd_vel to send command to the mobile robot
    ros::Publisher pub_cmd_vel;

    // communication with person_detector or person_tracker
    ros::Publisher pub_goal_reached;
    ros::Subscriber sub_goal_to_reach;

	// communication with odometry
    ros::Publisher pub_change_odometry;
    ros::Subscriber sub_odometry;
    
    int rotation;//boolean to check if there is a rotation to do
    int translation;//boolean to check if there is a translation to do

    float rotation_to_do;
    float rotation_done;
    float translation_to_do;
    float translation_done;
    geometry_msgs::Point goal_to_reach;
    geometry_msgs::Point my_position;
    float kp ; 
    float ki ; 
    float kd ; 
    float ep ;
    float ei ; 
    float ed ; 

public:

transrot() {

    // communication with cmd_vel
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    // communication with person_detector or person_tracker
    pub_goal_reached = n.advertise<geometry_msgs::Point>("goal_reached", 1);
    sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &transrot::goal_to_reachCallback, this);
	
	// communication with odometry
    pub_change_odometry = n.advertise<geometry_msgs::Point>("change_odometry", 1);
    sub_odometry = n.subscribe("odom", 1, &transrot::odomCallback, this);


    rotation_to_do= 0;
    rotation_done= 0;
    translation_to_do= 0;
    translation_done= 0;
   translation = 1;
   rotation = 1;
	
    kp = 0.5;
    ki = 0.0001;
    kd = 0.01;
    ep = 0.0;
    ed = 0.0;
    ei = 0.0;

}

void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {
	
	if(!translation && !rotation )
	return;
	rotation_done = tf::getYaw(o->pose.pose.orientation);
    translation_done = o->pose.pose.position.x;
	int rflag =0;
	int tflag=0;



        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;

        //ROS_INFO("(translation_node) translation_done: %f, translation_to_do: %f", translation_done, translation_to_do);
        my_position. x += translation_done*cos(rotation_done);
        my_position.y += translation_done*sin(rotation_done);
        translation_to_do = sqrt(pow((goal_to_reach.x - my_position.x ),2) +  pow((goal_to_reach.y - my_position.y ),2));
        rotation_to_do = acos( (goal_to_reach.x - my_position.x)/ translation_to_do ) - rotation_done;

        
        if (fabs(rotation_to_do - rotation_done)> rotation_threshold)  {
         
         
         
         // implementation of a PID controller
          
         	ed = ed - (rotation_to_do - rotation_done);
            ep = (rotation_to_do - rotation_done);//the current error is the difference between the rotation_to_do (ie, the angle to reach) and the rotation_done (ie, the current angle)
			ei +=ep;
			twist.angular.z = kp * ep + ki * ei + kd * ed;
            
            ROS_INFO("Twist angle in rotation %f ", twist.angular.z*180/M_PI);
			rflag=1;
        
        }

        if (( fabs(translation_to_do - translation_done) > translation_threshold ) && fabs(rotation_to_do - rotation_done) < 0.35) {
                         
             
             // implementation of a PID controller
          
            ed = ed - (translation_to_do - translation_done);//the current error is the difference between the rotation_to_do (ie, the angle to reach) and the rotation_done (ie, the current angle)
            
            ep = (translation_to_do - translation_done);//the current error is the difference between the rotation_to_do (ie, the angle to reach) and the rotation_done (ie, the current angle)
            ei +=ep;
            twist.linear.x = kp * ep + ki * ei + kd * ed;
			tflag=1;
            


        }
        pub_cmd_vel.publish(twist);
        if(!( tflag||rflag ))
        {
			translation = 0;
			rotation = 0;
		//the translation is done so we send the goal_reached to the detector/tracker node
		geometry_msgs::Point msg_goal_reached;

		//to complete
		pub_goal_reached.publish(msg_goal_reached);	
		
		}
                   

}

void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from the person tracker
// we decompose the goal_received in one rotation and one translation
// and we perform the rotation first

    if ( !rotation && !translation ) {//we do not accept new goal if the current goal is not reached
        

        goal_to_reach.x = g->x;
        goal_to_reach.y = g->y;
        
        my_position.x = 0;
        my_position.y = 0;

        ROS_INFO("(transrot_node) goal_to_reach (%f, %f)", goal_to_reach.x, goal_to_reach.y);
        //getchar();

        //we compute the translation_to_do
        translation = 1;
        translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

        //we compute the rotation_to_do
        rotation = 1;
        rotation_to_do = acos( goal_to_reach.x / translation_to_do );

        if ( goal_to_reach.y < 0 )
            rotation_to_do *=-1;

        ROS_INFO("(transrot_node) rotation_to_do: %f", rotation_to_do*180/M_PI);
	
		//reset odometry
		geometry_msgs::Point p;
		p.x = 0.0;
		p.y = 0.0;
		p.z = 0.0;
		pub_change_odometry.publish(p);
		ros::Duration(0.5).sleep();
        

    }

}





};

int main(int argc, char **argv){

    ros::init(argc, argv, "transrot");

    transrot bsObject;

    ros::spin();

    return 0;
}
