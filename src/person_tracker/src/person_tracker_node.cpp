// person detector using lidar data
// written by O. Aycard

#include "ros/ros.h"
#include "ros/time.h"
// We use the already built messages types
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Float32.h"


#define cluster_threshold 0.15
#define static_threshold 0.15
#define dynamic_threshold 0.8
#define least_leg_size 0.03
#define biggest_leg_size 0.2
#define timeout 5
using namespace std;

class person_tracker {

private:
    ros::NodeHandle n;
    ros::Subscriber sub_scan;
    ros::Publisher pub_person_tracker;
    ros::Publisher pub_goal_to_reach;
    ros::Subscriber sub_goal_reached;

    // to store, process and display the laserdata
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float range[1000];
    geometry_msgs::Point current_scan[1000];
    std_msgs::ColorRGBA current_colors[1000];

    //to perform detection
    float background[1000];// to store the background
    int detection[1000];// to store if the current hit of the laser is static or dynamic
    int first_scan;// to know if this is the first scan or not

    //to perform clustering
    int nb_cluster;// number of cluster
    int cluster[1000]; //to store for each hit, the cluster it belongs to
    float size_cluster[1000];// to store the size of each cluster
    geometry_msgs::Point center_cluster[1000];// to store the center of gravity of each cluster
    float dynamic_cluster[1000];// to store the percentage of the cluster that is dynamic

    //to perform search of moving legs and to store them
    int nb_moving_leg;
    int moving_leg[1000];//we store the cluster corresponding to a moving leg

    //to perform search of moving persons and to store them
    int nb_moving_person;
    geometry_msgs::Point position_moving_person[1000];

    int goal_reached;
    int timestamp_count;

public:

person_tracker() {

    pub_person_tracker = n.advertise<visualization_msgs::Marker>("person_tracker", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

    sub_scan = n.subscribe("scan", 1, &person_tracker::scanCallback, this);

    // communication with control_node
    pub_goal_to_reach = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.
    sub_goal_reached = n.subscribe("goal_reached", 1, &person_tracker::goal_reachedCallback, this);

    goal_reached = 1;
    first_scan = 1;
    timestamp_count = 0;

}

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));
    // return abs(pa.x-pb.x) + abs(pa.y-pb.y);

}

// Draw the field of view and other references
void populateMarkerReference() {

    visualization_msgs::Marker references;
	
    references.header.frame_id = "laser";
    references.header.stamp = ros::Time::now();
    references.ns = "person_tracker";
    references.id = 1;
    references.type = visualization_msgs::Marker::LINE_STRIP;
    references.action = visualization_msgs::Marker::ADD;
    references.pose.orientation.w = 1;

    references.scale.x = 0.02;

    references.color.r = 1.0f;
    references.color.g = 1.0f;
    references.color.b = 1.0f;
    references.color.a = 1.0;
    geometry_msgs::Point v;

    v.x =  0.02 * cos(-2.356194);
    v.y =  0.02 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  5.6 * cos(-2.356194);
    v.y =  5.6 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    float beam_angle = -2.356194 + 0.006136;
    // first and last beam are already included
    for (int i=0 ; i< 723; i++, beam_angle += 0.006136){
        v.x =  5.6 * cos(beam_angle);
        v.y =  5.6 * sin(beam_angle);
        v.z = 0.0;
        references.points.push_back(v);
    }

    v.x =  5.6 * cos(2.092350);
    v.y =  5.6 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  0.02 * cos(2.092350);
    v.y =  0.02 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    pub_person_tracker.publish(references);

}

void populateMarkerTopic(int nb_beams, geometry_msgs::Point *current_scan, std_msgs::ColorRGBA *current_colors){

    visualization_msgs::Marker marker;

    //ROS_INFO("entree dans marker topic");
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "person_tracker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    //marker.pose.position.x = 1;
    //marker.pose.position.y = 1;
    //marker.pose.position.z = 1;
    //marker.pose.orientation.x = 0;
    //marker.pose.orientation.y = 0;
    //marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    //marker.scale.z = 2;

    //marker.color.g = 1.0f;
    marker.color.a = 1.0;

    //ROS_INFO("ok");
    for (int loop = 0; loop < nb_beams; loop++) {
        geometry_msgs::Point p;
        std_msgs::ColorRGBA c;

         p.x = current_scan[loop].x;
         p.y = current_scan[loop].y;
         p.z = current_scan[loop].z;

         c.r = current_colors[loop].r;
         c.g = current_colors[loop].g;
         c.b = current_colors[loop].b;
         c.a = current_colors[loop].a;

         marker.points.push_back(p);
         marker.colors.push_back(c);

        }

        pub_person_tracker.publish(marker);
        populateMarkerReference();

}

void store_background() {
// store all the hits of the laser in the background table

    ROS_INFO("store_background");
    for(int loop = 0; loop< 1000; loop++)
    {
        background[loop]= range[loop];
    }

}

void detect_motion() {
// for each hit, compare the current range with the background to detect motion

    ROS_INFO("detect_motion");
    for(int loop = 0; loop<1000; loop++)
    {
        if (abs(range[loop]- background[loop]) > static_threshold)
        {
            detection[loop] = 1;
        }
        else
        {
            detection[loop]=0;
        }
       //background[loop]=range[loop];
    }
    //populateMarkerTopic(nb_beams, current_scan, current_colors);
    //getchar();

}

void clustering() {
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit of the laser and the current one is higher than a threshold we put the current hit in the current cluster
//else we start a new cluster

    int start_cluster, end_cluster;// to store the start and end of each cluster
    ROS_INFO("clustering");
    float nb_dynamic = 0;// to store the number of "dynamic hits" in the current cluster
    nb_cluster = 0;
    start_cluster= 0;
    end_cluster=0;
    geometry_msgs::Point temp_cog;
    temp_cog.x = 0;
    temp_cog.y = 0;
	cluster[0] = nb_cluster;
	temp_cog.x = current_scan[0].x ;
    temp_cog.y = current_scan[0].y ;
	int points = 0;
    for(int loop = 1; loop <1000 ; loop++)
    {
    	
    	if (  distancePoints(current_scan[loop], current_scan[loop-1] ) > cluster_threshold  || loop == 1000 - 1  )
		{
			
			
			center_cluster[nb_cluster].x = temp_cog.x / max(1 , points);
			center_cluster[nb_cluster].y = temp_cog.y / max(1 , points);
			dynamic_cluster[nb_cluster] = nb_dynamic / max(1 , points);
			
			nb_cluster++;
			points = 1;
			temp_cog.x = current_scan[loop].x ;
    		temp_cog.y = current_scan[loop].y ;
			size_cluster[nb_cluster] = 0;	
			cluster[loop] = nb_cluster;				

			nb_dynamic = 0;
			if(detection[loop] == 1)
    			nb_dynamic += 1;


		}
    	else
    	{
    		//ROS_INFO("%d ", loop);
    		cluster[loop] = nb_cluster;
    		temp_cog.x += current_scan[loop].x ;
    		temp_cog.y += current_scan[loop].y ;
    		if(detection[loop] == 1)
    			nb_dynamic += 1;
    		size_cluster[nb_cluster] +=  distancePoints(current_scan[loop], current_scan[loop -1]);
    		points += 1;
    	}
    	
    	
    }
	/*for(int loop = 0; loop <nb_cluster ; loop++)
	{
		ROS_INFO("dynamic Cluster %f %f ", dynamic_cluster[loop], size_cluster[loop]);
	}
	*/
    //getchar();

}


void search_moving_leg() {

    ROS_INFO("search_moving_leg");
    nb_moving_leg = 0;

    //populateMarkerTopic(nb_beams, current_scan, current_colors);
    //if ( nb_moving_leg )
    //    getchar();
    for(int loop= 0; loop < nb_cluster; loop ++)
    {    	
    	//ROS_INFO("Cluster size %f dynamic size %f", size_cluster[loop], dynamic_cluster[loop]);
    	if (dynamic_cluster[loop] >=dynamic_threshold && size_cluster[loop] >=  least_leg_size && size_cluster[loop]<= biggest_leg_size )
    	{
    		//ROS_INFO("Moving Leg Detected");
    		moving_leg[nb_moving_leg] = loop;
    		nb_moving_leg++;

    	}	    	    
    }
    ROS_INFO("moving leg detected %d", nb_moving_leg);

}

void search_moving_person() {

    ROS_INFO("search_moving_person");
    nb_moving_person = 0;

    populateMarkerTopic(nb_beams, current_scan, current_colors);
    if ( nb_moving_person > 0 )
        getchar();

    int first_leg_cluster;
    int first_leg_extreme;
    int second_leg_cluster;
    int second_leg_extreme;
    if(nb_moving_leg == 0)
    	return;
    for (int loop= 0; loop<nb_moving_leg-1; loop++)
    {
    	first_leg_extreme = moving_leg[loop];
    	second_leg_extreme = moving_leg[loop+1];
    	//ROS_INFO("%d %d hi", first_leg_extreme, second_leg_extreme);
    	if (distancePoints(center_cluster[second_leg_extreme], center_cluster[first_leg_extreme]) < 25)
    	{	

    		position_moving_person[nb_moving_person].x = ( center_cluster[first_leg_extreme].x + center_cluster[second_leg_extreme].x )/2;
    	    position_moving_person[nb_moving_person].y = ( center_cluster[first_leg_extreme].y + center_cluster[second_leg_extreme].y )/2;
    	   // ROS_INFO("Moving Person Detected");
    	    nb_moving_person += 1;
    	    
    	}
    	
    } 

    ROS_INFO("Number of Moving Person Detected %d", nb_moving_person);
}

void choose_goal() {

    ROS_INFO("choose_goal");

    //populateMarkerTopic(nb_beams, current_scan, current_colors);
    //getchar();
    geometry_msgs::Point goal_to_reach;
    
    if(nb_moving_person == 1)
    {
		if(timestamp_count < timeout)
		{
			timestamp_count ++;
			goal_reached = 1;
			ros::Duration(0.1).sleep();
			store_background();
			return;
		}
    	goal_to_reach.x = position_moving_person[0].x;
    	goal_to_reach.y = position_moving_person[0].y;
        pub_goal_to_reach.publish(goal_to_reach);
        //exit(0);
    }
        
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    ROS_INFO("--Listening to scan:");
    ROS_INFO("goal_reached = %d", goal_reached);
    ROS_INFO("first_scan = %d", first_scan);

    if ( goal_reached ) {

        ROS_INFO("Timestamp: %d", scan->header.seq);

        // store the important data related to laserscanner
        range_min = scan->range_min;
        range_max = 5;//scan->range_max; we decide that the range is limited to 5 meters
        angle_min = scan->angle_min;
        angle_max = scan->angle_max;
        angle_inc = scan->angle_increment;
        nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;

        ROS_INFO("range_min, range_max: %f, %f", range_min, range_max);
        ROS_INFO("angle_min: %f", angle_min*180/M_PI);
        ROS_INFO("angle_max: %f", angle_max*180/M_PI);
        ROS_INFO("angle_increment: %f", angle_inc*180/M_PI);
        ROS_INFO("number_of_beams: %d", nb_beams);

        // store the range and the coordinates in cartesian framework of each hit
        float beam_angle = angle_min;
        for (int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc) {
            if ( scan->ranges[loop] < range_max )
                range[loop] = scan->ranges[loop];
            else
                range[loop] = range_max;

            //transform the scan in cartesian framewrok
            current_scan[loop].x = range[loop] * cos(beam_angle);
            current_scan[loop].y = range[loop] * sin(beam_angle);
            current_scan[loop].z = 0.0;

            // green color by default for each hit
            if(!detection[loop])
            {
            	current_colors[loop].r = 0.0;
	            current_colors[loop].g = 1.0;
	            current_colors[loop].b = 0.0;
	            current_colors[loop].a = 1.0;
	        }
	        else
	        {
	        	current_colors[loop].r = 1.0;
	            current_colors[loop].g = 0.0;
	            current_colors[loop].b = 0.0;
	            current_colors[loop].a = 1.0;
	        }

        }

        if ( first_scan ) {

            store_background();
            first_scan = 0;

        }
        else {

            detect_motion();
            clustering();
            search_moving_leg();
            search_moving_person();
            choose_goal();

        }
    }
}

void goal_reachedCallback(const geometry_msgs::Point::ConstPtr& g) {
    first_scan = 1;
    goal_reached = 1;
    store_background();
    timestamp_count = 0;

}

};


int main(int argc, char **argv){

    ros::init(argc, argv, "person_tracker");

    person_tracker bsObject;

    ros::spin();

    return 0;
}


