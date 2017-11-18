#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#define translation_threshold 0.4
#define big_rotation_threshold 0.8
#define small_rotation_threshold 0.1
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
    
    //communication with obstacle
    ros::Subscriber sub_obstacle_float;
    
    int rotation;//boolean to check if there is a rotation to do
    int translation;//boolean to check if there is a translation to do

    float rotation_to_do;
    float rotation_done;
    float translation_to_do;
    float translation_done;
    float obstacle_float;
    geometry_msgs::Point goal_to_reach;
    geometry_msgs::Point my_position;
    float kp ; 
    float ki ; 
    float kd ; 
    float epr ;
    float eir ; 
    float edr ;
    float ept ;
    float eit ;
    float edt ; 
    float prevorient;
    int crotate;
    int ctranslate;
    int action;

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
    
    //communication with obstacle detection
    sub_obstacle_float = n.subscribe("obstacle_float",1, &transrot::ObstacleCallBack, this);
     
    rotation_to_do= 0;
    rotation_done= 0;
    translation_to_do= 0;
    translation_done= 0;
    translation = 0;
    rotation = 0;
    
    kp = 0.7;
    ki = 0.0;
    kd = 0.0;
    epr = 0.0;
    edr = 0.0;
    eir = 0.0;
    ept = 0.0;
    edt = 0.0;
    eit = 0.0;
    prevorient = 0.0;
    crotate = 0;
    ctranslate = 0;
    action = 0;

}
void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {
    if(!action && obstacle_float == 0.0 )
        return;
    int flag = 0;
    my_position.x =o->pose.pose.position.x;
    my_position.y =o->pose.pose.position.y;
    
    translation_to_do = sqrt(pow((goal_to_reach.x - my_position.x ),2) +  pow((goal_to_reach.y - my_position.y ),2));
    rotation_to_do = acos( (goal_to_reach.x - my_position.x)/ translation_to_do ) ;

    if(rotation_to_do < 0)
        rotation_to_do *= -1;
    
    rotation_done = tf::getYaw(o->pose.pose.orientation); 
    translation_done = sqrt(pow((my_position.x ),2) +  pow((my_position.y ),2));
    
    

    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    if (fabs(rotation_to_do -rotation_done )> big_rotation_threshold )  { 
        epr = (rotation_to_do -rotation_done );
        twist.angular.z = 0.9 * epr + ki * eir + kd * edr;
        flag = 1;
    }

    if (fabs(translation_to_do-translation_done) > translation_threshold ){
        ept = (translation_to_do-translation_done);
        if(obstacle_float != 1.0 && fabs(rotation_to_do - rotation_done )<big_rotation_threshold){
            twist.linear.x = 0.65 * ept + ki * eit + kd * edt;
            epr = (rotation_to_do -rotation_done );
            twist.angular.z = 0.6 * epr + ki * eir + kd * edr;
            flag = 1;

        }
    }
    ROS_INFO("%f rotation_to_do ", rotation_to_do);
    ROS_INFO("%f translation_to_do ", translation_to_do);
    if(flag && obstacle_float == 0.0)
        pub_cmd_vel.publish(twist);
    else
       { 
        action = 0;
        geometry_msgs::Point msg_goal_reached;

        pub_goal_reached.publish(msg_goal_reached); 

       }
    ros::Duration(0.1).sleep();
    
}

/*
void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {
    
    if(!translation && !rotation )
    return;

    rotation_done = tf::getYaw(o->pose.pose.orientation); 
    ROS_INFO ("rotation_done %f", rotation_done*180/M_PI);
    
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;


    my_position.x =o->pose.pose.position.x;
    my_position.y =o->pose.pose.position.y;
    ROS_INFO ("x done %f", my_position.x );
    ROS_INFO ("y done %f", my_position.y );

    if(obstacle_float == 1.0){
        translation_to_do = 0;
        rotation_to_do = 0;
        ctranslate = 0;
        crotate = 0;

    }
    else if(ctranslate)
    {
        //prevorient = rotation_done;
        translation_to_do = sqrt(pow((goal_to_reach.x - my_position.x ),2) +  pow((goal_to_reach.y - my_position.y ),2));
        rotation_to_do = acos( (goal_to_reach.x - my_position.x)/ translation_to_do ) ;

        if(rotation_done < 0)
            rotation_done *= -1;

        //rotation_to_do +=  rotation_done;
        if (goal_to_reach.y    < 0 )
            rotation_to_do *=-1;
        rotation_to_do = rotation_to_do - rotation_done;
        
        
    }
    else if (crotate && !ctranslate) 
    {
        rotation_to_do -= rotation_done;
    }

  //if finished go to goal reached
    if(fabs(rotation_to_do ) < small_rotation_threshold &&  fabs(translation_to_do ) < translation_threshold )
    {
        epr = 0.0;
        edr = 0.0;
        eir = 0.0;
        ept = 0.0;
        edt = 0.0;
        eit = 0.0;
        //the translation is done so we send the goal_reached to the detector/tracker node
        geometry_msgs::Point msg_goal_reached;

        //to complete
        

        translation = 0;
        rotation = 0;
        pub_goal_reached.publish(msg_goal_reached); 
        return;
        //ros::Duration(1.0).sleep();
    }

    
   if (fabs(rotation_to_do  )> big_rotation_threshold )  { //only rotate don't translate        

        ROS_INFO("BIG rotation_to_do %f ",rotation_to_do*180/M_PI);
    
        epr = (rotation_to_do );
        twist.angular.z = kp * epr + ki * eir + kd * edr;
        crotate = 1;
        
        
    }
    else if (fabs(rotation_to_do  ) <= big_rotation_threshold )  {
        

        if(fabs(rotation_to_do) < small_rotation_threshold)
        {
            rotation_to_do = 0;
            crotate = 0;
        }
        else 
            crotate = 1;

        ROS_INFO("SMALL rotation_to_do %f ", rotation_to_do*180/M_PI);
        epr = (rotation_to_do);
        twist.angular.z = kp * epr + ki * eir + kd * edr;

        if (fabs(translation_to_do) > translation_threshold ){
            ROS_INFO("translation_to_do %f ", translation_to_do);

            ept = (translation_to_do );
            twist.linear.x = kp * ept + ki * eit + kd * edt;
            ctranslate = 1;
        
        }
        else{
            ctranslate = 0;
            translation_to_do = 0;
        }

    }
    else
        crotate = 0;
    //getchar();
    pub_cmd_vel.publish(twist);
    //ros::Duration(0.05).sleep();
    
}
*/

void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from the person tracker
// we decompose the goal_received in one rotation and one translation
// and we perform the rotation first

   // if ( !rotation && !translation ) {//we do not accept new goal if the current goal is not reached
    if(1){
    goal_to_reach.x = g->x;
    goal_to_reach.y = g->y;
    
    //my_position.x = 0;
    //my_position.y = 0;

    ROS_INFO("(transrot_node) goal_to_reach (%f, %f)", goal_to_reach.x, goal_to_reach.y);
    

            //reset odometry
    geometry_msgs::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    pub_change_odometry.publish(p);

        
/*    translation = 1;
    rotation = 1;
    crotate = 1;
    ctranslate = 0;
    prevorient = 0.0; */
    action = 1;
    }

}


void ObstacleCallBack(const std_msgs::Float32::ConstPtr & a){

    obstacle_float = a->data;
    ROS_INFO("Obtacle in transrot %f ",obstacle_float);
}


};

int main(int argc, char **argv){

    ros::init(argc, argv, "transrot");

    transrot bsObject;

    ros::spin();

    return 0;
}